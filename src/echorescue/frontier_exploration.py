"""Deterministic, ROS-independent occupancy mapping and frontier planning."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from math import cos, floor, isfinite, radians, sin
from typing import Any

from echorescue.models import CellState, Position
from echorescue.planner_flight import Cell, GridTransform, inflate_occupied
from echorescue.planning import astar
from echorescue.sensor_replanning import PoseSample, RangeObservation


class ExplorationState(str, Enum):
    INITIALIZING = "initializing"
    TAKEOFF = "takeoff"
    MAPPING = "mapping"
    SELECTING_FRONTIER = "selecting_frontier"
    NAVIGATING = "navigating"
    SETTLING = "settling"
    REPLANNING = "replanning"
    EXPLORATION_COMPLETE = "exploration_complete"
    RETURNING = "returning"
    LANDING = "landing"
    RECOVERY = "recovery"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class FrontierPolicy:
    minimum_cluster_size: int = 1
    information_radius_cells: int = 2
    information_gain_weight: float = 2.0
    path_cost_weight: float = 1.0
    target_switch_penalty: float = 1.0

    def __post_init__(self) -> None:
        values = (self.information_gain_weight, self.path_cost_weight, self.target_switch_penalty)
        if self.minimum_cluster_size < 1 or self.information_radius_cells < 1:
            raise ValueError("frontier cluster and information radius limits must be positive")
        if not all(isfinite(value) and value >= 0 for value in values):
            raise ValueError("frontier scoring weights must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class MappingPolicy:
    transform: GridTransform
    footprint_m: float
    sensor_frame: str = "iris_range_link"
    sensor_yaw_offset_enu_deg: float = 0.0
    range_min_m: float = 0.25
    range_max_m: float = 4.0
    horizontal_field_of_view_rad: float = 6.283185307179586
    observation_max_age_s: float = 0.5
    pose_max_age_s: float = 0.5
    pose_observation_max_skew_s: float = 0.2
    maximum_map_revisions: int = 500

    def __post_init__(self) -> None:
        values = (
            self.footprint_m, self.range_min_m, self.range_max_m,
            self.horizontal_field_of_view_rad, self.observation_max_age_s,
            self.pose_max_age_s, self.pose_observation_max_skew_s,
        )
        if not all(isfinite(value) and value > 0 for value in values):
            raise ValueError("mapping distances and freshness limits must be positive")
        if self.range_min_m >= self.range_max_m or self.maximum_map_revisions < 1:
            raise ValueError("mapping range or revision limit is invalid")


@dataclass(frozen=True, slots=True)
class FrontierCandidate:
    cell: Cell
    cluster: tuple[Cell, ...]
    path: tuple[Cell, ...]
    path_cost: int
    information_gain: int
    switch_penalty: float
    score: float

    def report(self) -> dict[str, Any]:
        return {
            **asdict(self),
            "cell": list(self.cell),
            "cluster": [list(cell) for cell in self.cluster],
            "path": [list(cell) for cell in self.path],
        }


def _neighbors(cell: Cell) -> tuple[Cell, ...]:
    row, column = cell
    return ((row, column + 1), (row + 1, column), (row, column - 1), (row - 1, column))


def _to_position(cell: Cell) -> Position:
    return Position(cell[1], cell[0])


def _from_position(position: Position) -> Cell:
    return position.y, position.x


class FrontierOccupancyMap:
    """Monotone UNKNOWN/FREE/OCCUPIED map derived only from validated beams."""

    def __init__(self, policy: MappingPolicy) -> None:
        self.policy = policy
        self._states = [CellState.UNKNOWN] * (policy.transform.rows * policy.transform.columns)
        self._occupied: set[Cell] = set()
        self._inflated: frozenset[Cell] = frozenset()
        self.pose: PoseSample | None = None
        self.session_id = ""
        self.last_sequence = 0
        self.last_sensor_time_ns = -1
        self.map_revision = 0
        self.accepted_scans = self.rejected_scans = 0
        self.accepted_beams = self.rejected_beams = 0
        self.rejections: list[dict[str, Any]] = []
        self.revisions: list[dict[str, Any]] = []

    def _index(self, cell: Cell) -> int:
        return cell[0] * self.policy.transform.columns + cell[1]

    def state(self, cell: Cell) -> CellState:
        if not self.policy.transform.contains(cell):
            return CellState.OCCUPIED
        return self._states[self._index(cell)]

    def observe_pose(self, pose: PoseSample) -> bool:
        if not all(isfinite(value) for value in (pose.east_m, pose.north_m, pose.heading_enu_deg)):
            self.rejections.append({"kind": "pose", "reason": "non-finite vehicle pose"})
            return False
        if self.pose and pose.session_id == self.pose.session_id and (
            pose.source_time_boot_ms <= self.pose.source_time_boot_ms
            or pose.receipt_monotonic_ns <= self.pose.receipt_monotonic_ns
        ):
            self.rejections.append({"kind": "pose", "reason": "vehicle-time regression"})
            return False
        if self.pose and pose.session_id != self.pose.session_id:
            self.last_sequence = 0
            self.last_sensor_time_ns = -1
        self.pose = pose
        self.session_id = pose.session_id
        return True

    def _reject(self, observation: RangeObservation, reason: str) -> None:
        self.rejected_scans += 1
        self.rejections.append({
            "kind": "scan", "sequence": observation.sequence,
            "session_id": observation.session_id, "reason": reason,
        })

    def _ray_cells(self, east: float, north: float, angle: float, distance: float) -> tuple[Cell, ...]:
        step = self.policy.transform.resolution_m / 4.0
        count = max(1, int(distance / step))
        cells: list[Cell] = []
        for index in range(count + 1):
            travelled = min(distance, index * step)
            point = east + travelled * cos(angle), north + travelled * sin(angle)
            try:
                cell = self.policy.transform.enu_to_cell(point)
            except ValueError:
                if cells:
                    break
                continue
            if not cells or cells[-1] != cell:
                cells.append(cell)
        return tuple(cells)

    def process(self, observation: RangeObservation, *, launch_east_m: float,
                launch_north_m: float, now_ns: int) -> bool:
        pose = self.pose
        checks = (
            (pose is None, "no associated vehicle pose"),
            (pose is not None and observation.session_id != pose.session_id, "cross-session observation"),
            (observation.sequence <= self.last_sequence, "duplicate or out-of-order sequence"),
            (observation.sensor_time_ns <= self.last_sensor_time_ns, "out-of-order sensor time"),
            (now_ns < observation.receipt_monotonic_ns or (now_ns - observation.receipt_monotonic_ns) / 1e9 > self.policy.observation_max_age_s, "stale or future observation"),
            (pose is not None and (now_ns < pose.receipt_monotonic_ns or (now_ns - pose.receipt_monotonic_ns) / 1e9 > self.policy.pose_max_age_s), "stale or future associated pose"),
            (pose is not None and abs(observation.receipt_monotonic_ns - pose.receipt_monotonic_ns) / 1e9 > self.policy.pose_observation_max_skew_s, "pose/observation skew exceeded"),
            (observation.sensor_frame != self.policy.sensor_frame, "unexpected sensor frame"),
            (not observation.ranges_m, "empty scan"),
            (not all(isfinite(value) for value in (observation.angle_min_rad, observation.angle_increment_rad, observation.range_min_m, observation.range_max_m)), "non-finite scan metadata"),
            (observation.range_min_m != self.policy.range_min_m or observation.range_max_m != self.policy.range_max_m, "invalid range limits"),
            (observation.angle_increment_rad <= 0 or observation.angle_increment_rad * max(0, len(observation.ranges_m) - 1) > self.policy.horizontal_field_of_view_rad + 1e-6, "invalid angle metadata"),
            (not all(isfinite(value) for value in observation.ranges_m), "non-finite range"),
            (any(value < observation.range_min_m for value in observation.ranges_m), "below-minimum range"),
            (any(value > observation.range_max_m for value in observation.ranges_m), "above-maximum range"),
        )
        for failed, reason in checks:
            if failed:
                self._reject(observation, reason)
                return False
        assert pose is not None
        self.last_sequence = observation.sequence
        self.last_sensor_time_ns = observation.sensor_time_ns
        self.accepted_scans += 1
        changed_free: set[Cell] = set()
        changed_occupied: set[Cell] = set()
        relative_east = pose.east_m - launch_east_m
        relative_north = pose.north_m - launch_north_m
        yaw = radians(pose.heading_enu_deg + self.policy.sensor_yaw_offset_enu_deg)
        for index, distance in enumerate(observation.ranges_m):
            self.accepted_beams += 1
            angle = yaw + observation.angle_min_rad + index * observation.angle_increment_rad
            hit = distance < observation.range_max_m - 1e-9
            cells = self._ray_cells(relative_east, relative_north, angle, distance)
            if not cells:
                self.rejected_beams += 1
                continue
            endpoint_inside = True
            try:
                endpoint = self.policy.transform.enu_to_cell((
                    relative_east + distance * cos(angle),
                    relative_north + distance * sin(angle),
                ))
            except ValueError:
                endpoint_inside = False
                endpoint = cells[-1]
            effective_hit = hit and endpoint_inside
            free_cells = cells[:-1] if effective_hit else cells
            for cell in free_cells:
                if self.state(cell) is CellState.UNKNOWN:
                    self._states[self._index(cell)] = CellState.FREE
                    changed_free.add(cell)
            if effective_hit:
                if self.state(endpoint) is not CellState.OCCUPIED:
                    self._states[self._index(endpoint)] = CellState.OCCUPIED
                    self._occupied.add(endpoint)
                    changed_free.discard(endpoint)
                    changed_occupied.add(endpoint)
        if not changed_free and not changed_occupied:
            return False
        if self.map_revision >= self.policy.maximum_map_revisions:
            raise RuntimeError("map revision budget exhausted")
        if changed_occupied:
            self._inflated = inflate_occupied(
                frozenset(self._occupied), self.policy.transform, self.policy.footprint_m,
            )
        self.map_revision += 1
        counts = self.counts()
        self.revisions.append({
            "map_revision": self.map_revision, "sequence": observation.sequence,
            "session_id": observation.session_id,
            "new_free_cells": [list(cell) for cell in sorted(changed_free)],
            "new_occupied_cells": [list(cell) for cell in sorted(changed_occupied)],
            **counts,
        })
        return True

    @property
    def occupied(self) -> frozenset[Cell]:
        return frozenset(self._occupied)

    @property
    def inflated(self) -> frozenset[Cell]:
        return self._inflated

    def counts(self) -> dict[str, int]:
        states = tuple(self._states)
        return {
            "unknown_cell_count": states.count(CellState.UNKNOWN),
            "free_cell_count": states.count(CellState.FREE),
            "occupied_cell_count": states.count(CellState.OCCUPIED),
            "inflated_cell_count": len(self.inflated),
        }

    def safe_free(self, cell: Cell) -> bool:
        return self.state(cell) is CellState.FREE and cell not in self.inflated

    def frontier_cells(self) -> tuple[Cell, ...]:
        return tuple(
            cell for cell in (
                (row, column)
                for row in range(self.policy.transform.rows)
                for column in range(self.policy.transform.columns)
            )
            if self.safe_free(cell) and any(
                self.policy.transform.contains(neighbor)
                and self.state(neighbor) is CellState.UNKNOWN
                for neighbor in _neighbors(cell)
            )
        )

    def frontier_clusters(self, minimum_size: int = 1) -> tuple[tuple[Cell, ...], ...]:
        remaining = set(self.frontier_cells())
        clusters: list[tuple[Cell, ...]] = []
        while remaining:
            seed = min(remaining)
            pending = [seed]
            cluster: set[Cell] = set()
            remaining.remove(seed)
            while pending:
                cell = pending.pop()
                cluster.add(cell)
                for neighbor in _neighbors(cell):
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        pending.append(neighbor)
            if len(cluster) >= minimum_size:
                clusters.append(tuple(sorted(cluster)))
        return tuple(sorted(clusters, key=lambda item: item[0]))

    def information_gain(self, cell: Cell, radius: int) -> int:
        return sum(
            self.state((row, column)) is CellState.UNKNOWN
            for row in range(max(0, cell[0] - radius), min(self.policy.transform.rows, cell[0] + radius + 1))
            for column in range(max(0, cell[1] - radius), min(self.policy.transform.columns, cell[1] + radius + 1))
        )

    def path(self, start: Cell, goal: Cell) -> tuple[Cell, ...] | None:
        result = astar(
            _to_position(start), _to_position(goal),
            lambda position: self.policy.transform.contains(_from_position(position))
            and self.safe_free(_from_position(position)),
        )
        return None if result is None else tuple(_from_position(position) for position in result)

    def candidates(self, start: Cell, policy: FrontierPolicy,
                   previous_target: Cell | None = None) -> tuple[FrontierCandidate, ...]:
        candidates: list[FrontierCandidate] = []
        for cluster in self.frontier_clusters(policy.minimum_cluster_size):
            # Prefer the deterministic highest-gain member and only invoke A*
            # until a reachable representative is found. Running a complete A*
            # search for every cell in a long frontier can starve the live scan
            # callback on fine maps without changing the selection policy.
            representative: tuple[Cell, tuple[Cell, ...]] | None = None
            for cell in sorted(
                cluster,
                key=lambda item: (-self.information_gain(item, policy.information_radius_cells), item),
            ):
                path = self.path(start, cell)
                if path is not None:
                    representative = cell, path
                    break
            if representative is None:
                continue
            cell, path = representative
            cost = len(path) - 1
            gain = self.information_gain(cell, policy.information_radius_cells)
            penalty = policy.target_switch_penalty if previous_target is not None and cell != previous_target else 0.0
            score = policy.information_gain_weight * gain - policy.path_cost_weight * cost - penalty
            candidates.append(FrontierCandidate(cell, cluster, path, cost, gain, penalty, score))
        return tuple(sorted(candidates, key=lambda item: (-item.score, item.cell[0], item.cell[1])))

    def select_frontier(self, start: Cell, policy: FrontierPolicy,
                        previous_target: Cell | None = None) -> FrontierCandidate | None:
        candidates = self.candidates(start, policy, previous_target)
        return candidates[0] if candidates else None

    def exploration_complete(self, start: Cell, policy: FrontierPolicy) -> bool:
        return not self.candidates(start, policy)

    def known_ratio(self) -> float:
        counts = self.counts()
        total = self.policy.transform.rows * self.policy.transform.columns
        return (counts["free_cell_count"] + counts["occupied_cell_count"]) / total

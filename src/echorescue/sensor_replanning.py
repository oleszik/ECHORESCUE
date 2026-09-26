"""ROS-independent incremental range mapping and bounded repeated replanning."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, radians, sin
from time import monotonic_ns
from typing import Any

from echorescue.models import Position
from echorescue.planner_flight import Cell, GridTransform, KnownMapPlan, compact_path, inflate_occupied
from echorescue.planning import astar
from echorescue.waypoint_mission import RelativeTarget


@dataclass(frozen=True, slots=True)
class RangeObservation:
    session_id: str
    sequence: int
    sensor_time_ns: int
    receipt_monotonic_ns: int
    sensor_frame: str
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges_m: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class PoseSample:
    session_id: str
    source_time_boot_ms: int
    receipt_monotonic_ns: int
    east_m: float
    north_m: float
    heading_enu_deg: float
    roll_rad: float = 0.0
    pitch_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class ReplanningLimits:
    observation_max_age_s: float = 0.5
    pose_max_age_s: float = 0.5
    pose_observation_max_skew_s: float = 0.2
    maximum_attempts: int = 2
    timeout_s: float = 0.25
    sensor_yaw_offset_enu_deg: float = 0.0
    sensor_frame: str = "iris_range_link"
    range_min_m: float = 0.25
    range_max_m: float = 4.0
    horizontal_field_of_view_rad: float = 6.283185307179586
    maximum_replans_per_target: int = 1
    cooldown_s: float = 0.5
    maximum_consecutive_failures: int = 1
    map_change_threshold_cells: int = 1

    def __post_init__(self) -> None:
        positive = (
            self.observation_max_age_s, self.pose_max_age_s,
            self.pose_observation_max_skew_s, self.timeout_s,
            self.range_min_m, self.range_max_m,
            self.horizontal_field_of_view_rad,
        )
        if not all(isfinite(value) and value > 0 for value in positive):
            raise ValueError("replanning time, range, and field-of-view limits must be positive")
        if self.range_min_m >= self.range_max_m or self.cooldown_s < 0:
            raise ValueError("replanning range or cooldown limits are invalid")
        if min(self.maximum_attempts, self.maximum_replans_per_target,
               self.maximum_consecutive_failures, self.map_change_threshold_cells) < 1:
            raise ValueError("replanning count limits must be positive")


def _grid_path(
    start: Cell, goal: Cell, transform: GridTransform, blocked: frozenset[Cell],
) -> tuple[Cell, ...] | None:
    result = astar(
        Position(start[1], start[0]), Position(goal[1], goal[0]),
        lambda p: transform.contains((p.y, p.x)) and (p.y, p.x) not in blocked,
    )
    return None if result is None else tuple((item.y, item.x) for item in result)


class SensorMapReplanner:
    """Validate scans, update known occupancy, and replan only unsafe remainders."""

    def __init__(self, plan: KnownMapPlan, limits: ReplanningLimits) -> None:
        self.plan = plan
        self.limits = limits
        self.pose: PoseSample | None = None
        self.occupied = set(plan.occupied)
        self.inflated = set(plan.inflated_occupied)
        self.last_sequence = 0
        self.last_sensor_time_ns = -1
        self.attempts = 0
        self.successful_replans = 0
        self.consecutive_failures = 0
        self.map_revision = 0
        self.route_generation = 1
        self.last_replan_ns: int | None = None
        self.attempts_by_target: dict[str, int] = {}
        self.observations: list[dict[str, Any]] = []
        self.pose_rejections: list[dict[str, Any]] = []
        self.map_updates: list[dict[str, Any]] = []
        self.replans: list[dict[str, Any]] = []
        self.skipped_replans: list[dict[str, Any]] = []
        self.failure_reason: str | None = None

    def observe_pose(self, pose: PoseSample) -> bool:
        if self.pose and pose.session_id == self.pose.session_id and (
            pose.source_time_boot_ms <= self.pose.source_time_boot_ms
            or pose.receipt_monotonic_ns <= self.pose.receipt_monotonic_ns
        ):
            self.pose_rejections.append({"session_id": pose.session_id, "reason": "vehicle-time regression"})
            return False
        if self.pose and pose.session_id != self.pose.session_id:
            self.last_sequence = 0
            self.last_sensor_time_ns = -1
            self.failure_reason = "telemetry session changed; route correlation invalid"
        if all(isfinite(value) for value in (pose.east_m, pose.north_m, pose.heading_enu_deg)):
            self.pose = pose
            return True
        self.pose_rejections.append({"session_id": pose.session_id, "reason": "non-finite vehicle pose"})
        return False

    def _reject(self, observation: RangeObservation, reason: str) -> None:
        self.observations.append({
            "sequence": observation.sequence, "session_id": observation.session_id,
            "sensor_time_ns": observation.sensor_time_ns, "accepted": False, "reason": reason,
        })

    def process(
        self, observation: RangeObservation, *, launch_east_m: float, launch_north_m: float,
        remaining_route: tuple[Cell, ...], now_ns: int,
        target_id: str = "active-target", route_generation: int | None = None,
    ) -> tuple[RelativeTarget, ...] | None:
        pose = self.pose
        if pose is None:
            self._reject(observation, "no associated vehicle pose")
            return None
        checks = (
            (not observation.session_id or observation.session_id != pose.session_id, "cross-session observation"),
            (observation.sequence <= self.last_sequence, "duplicate or out-of-order sequence"),
            (observation.sensor_time_ns <= self.last_sensor_time_ns, "out-of-order sensor time"),
            (now_ns < observation.receipt_monotonic_ns or (now_ns - observation.receipt_monotonic_ns) / 1e9 > self.limits.observation_max_age_s, "stale or future observation"),
            (now_ns < pose.receipt_monotonic_ns or (now_ns - pose.receipt_monotonic_ns) / 1e9 > self.limits.pose_max_age_s, "stale or future associated pose"),
            (abs(observation.receipt_monotonic_ns - pose.receipt_monotonic_ns) / 1e9 > self.limits.pose_observation_max_skew_s, "pose/observation skew exceeded"),
            (observation.sensor_frame != self.limits.sensor_frame, "unexpected sensor frame"),
            (not all(isfinite(value) for value in (
                observation.angle_min_rad, observation.angle_increment_rad,
                observation.range_min_m, observation.range_max_m,
            )), "non-finite scan metadata"),
            (observation.range_min_m != self.limits.range_min_m
             or observation.range_max_m != self.limits.range_max_m
             or observation.range_min_m >= observation.range_max_m, "invalid range limits"),
            (not observation.ranges_m, "empty scan"),
            (observation.angle_increment_rad <= 0.0
             or observation.angle_increment_rad * max(0, len(observation.ranges_m) - 1) > self.limits.horizontal_field_of_view_rad + 1e-6,
             "invalid field of view"),
            (not all(isfinite(value) for value in observation.ranges_m), "non-finite range"),
            (any(value < observation.range_min_m for value in observation.ranges_m), "below-minimum range"),
            (any(value > observation.range_max_m for value in observation.ranges_m), "above-maximum range"),
        )
        for failed, reason in checks:
            if failed:
                self._reject(observation, reason)
                return None
        self.last_sequence = observation.sequence
        self.last_sensor_time_ns = observation.sensor_time_ns
        discovered: set[Cell] = set()
        yaw = radians(pose.heading_enu_deg + self.limits.sensor_yaw_offset_enu_deg)
        relative_east = pose.east_m - launch_east_m
        relative_north = pose.north_m - launch_north_m
        for index, measured_range in enumerate(observation.ranges_m):
            if measured_range >= observation.range_max_m - 1e-9:
                continue
            angle = yaw + observation.angle_min_rad + index * observation.angle_increment_rad
            # Mark the hit cell and one cell of unknown depth behind the first
            # return. A range sensor observes only a surface; assuming a
            # zero-thickness obstacle would not be conservative.
            hit_cells: list[Cell] = []
            resolution = self.plan.transform.resolution_m
            for distance in (measured_range, measured_range + resolution):
                point = (
                    relative_east + distance * cos(angle),
                    relative_north + distance * sin(angle),
                )
                try:
                    cell = self.plan.transform.enu_to_cell(point)
                except ValueError:
                    continue
                hit_cells.append(cell)
            # Range noise and cell quantisation can place a return just in
            # front of a known surface. Treat the return as explained when a
            # short continuation of the same ray reaches the effective map;
            # otherwise a known wall can become a false unknown obstacle.
            explained_by_map = False
            for distance in (measured_range, measured_range + resolution,
                             measured_range + 2.0 * resolution):
                point = (
                    relative_east + distance * cos(angle),
                    relative_north + distance * sin(angle),
                )
                try:
                    if self.plan.transform.enu_to_cell(point) in self.inflated:
                        explained_by_map = True
                        break
                except ValueError:
                    continue
            # Do not project unknown depth through a known surface.
            if hit_cells and not explained_by_map:
                discovered.update(cell for cell in hit_cells if cell not in self.inflated)
        self.observations.append({
            "sequence": observation.sequence, "session_id": observation.session_id,
            "sensor_time_ns": observation.sensor_time_ns, "accepted": True,
            "pose_source_time_boot_ms": pose.source_time_boot_ms,
            "discovered_cells": [list(cell) for cell in sorted(discovered)],
        })
        if not discovered:
            return None
        before = frozenset(self.inflated)
        self.occupied.update(discovered)
        updated = inflate_occupied(frozenset(self.occupied), self.plan.transform, self.plan.footprint_m)
        new_inflated = frozenset(updated - before)
        self.inflated = set(updated)
        self.map_revision += 1
        intersection = tuple(cell for cell in remaining_route if cell in updated)
        self.map_updates.append({
            "sequence": observation.sequence, "map_revision": self.map_revision,
            "session_id": observation.session_id,
            "pose": {"east_m": pose.east_m, "north_m": pose.north_m,
                     "source_time_boot_ms": pose.source_time_boot_ms},
            "new_occupied_cells": [list(cell) for cell in sorted(discovered)],
            "new_inflated_cells": [list(cell) for cell in sorted(new_inflated)],
            "remaining_route_intersection": [list(cell) for cell in intersection],
        })
        if not intersection:
            self.skipped_replans.append({"sequence": observation.sequence, "map_revision": self.map_revision,
                                         "reason": "map change irrelevant to remaining route"})
            return None
        generation = self.route_generation if route_generation is None else route_generation
        request = {"sequence": observation.sequence, "map_revision": self.map_revision,
                   "target_id": target_id, "route_generation": generation}
        if len(new_inflated) < self.limits.map_change_threshold_cells:
            self.failure_reason = "relevant map change below reconsideration threshold"
            self.skipped_replans.append({**request, "reason": self.failure_reason})
            return ()
        if self.attempts >= self.limits.maximum_attempts:
            self.failure_reason = "replan budget exhausted"
            self.replans.append({**request, "status": "SKIPPED", "reason": self.failure_reason})
            return ()
        if self.consecutive_failures >= self.limits.maximum_consecutive_failures:
            self.failure_reason = "consecutive planning failure budget exhausted"
            self.replans.append({**request, "status": "SKIPPED", "reason": self.failure_reason})
            return ()
        target_attempts = self.attempts_by_target.get(target_id, 0)
        if target_attempts >= self.limits.maximum_replans_per_target:
            self.failure_reason = "per-target replan budget exhausted"
            self.replans.append({**request, "status": "SKIPPED", "reason": self.failure_reason})
            return ()
        if self.last_replan_ns is not None and (now_ns - self.last_replan_ns) / 1e9 < self.limits.cooldown_s:
            self.failure_reason = "unsafe route changed during replan cooldown"
            self.skipped_replans.append({**request, "reason": self.failure_reason})
            return ()
        self.attempts += 1
        self.attempts_by_target[target_id] = target_attempts + 1
        self.last_replan_ns = now_ns
        started = monotonic_ns()
        try:
            current = self.plan.transform.enu_to_cell((relative_east, relative_north))
        except ValueError:
            self.failure_reason = "current vehicle position is outside the planner map"
            return ()
        outbound = _grid_path(current, self.plan.goal, self.plan.transform, updated)
        returning = _grid_path(self.plan.goal, self.plan.start, self.plan.transform, updated) if outbound else None
        elapsed_s = (monotonic_ns() - started) / 1e9
        if elapsed_s > self.limits.timeout_s:
            self.failure_reason = "bounded replanning timeout"
            self.consecutive_failures += 1
            self.replans.append({**request, "attempt": self.attempts, "status": "FAIL",
                                 "elapsed_s": elapsed_s, "reason": self.failure_reason})
            return ()
        if outbound is None or returning is None:
            self.failure_reason = "no safe route after sensor discovery"
            self.consecutive_failures += 1
            self.replans.append({**request, "attempt": self.attempts, "status": "FAIL", "reason": self.failure_reason})
            return ()
        outbound_compact = compact_path(outbound, updated)
        return_compact = compact_path(returning, updated)
        cells = outbound_compact[1:] + return_compact[1:]
        targets: list[RelativeTarget] = []
        for index, cell in enumerate(cells):
            east, north = self.plan.transform.cell_center_to_enu(cell)
            outbound_count = len(outbound_compact) - 1
            if index < outbound_count:
                target_id = f"g{generation + 1:03d}-outbound-goal" if index == outbound_count - 1 else f"g{generation + 1:03d}-outbound-{index + 1:03d}"
            else:
                return_index = index - outbound_count
                target_id = f"g{generation + 1:03d}-return-launch" if return_index == len(return_compact) - 2 else f"g{generation + 1:03d}-return-{return_index + 1:03d}"
            targets.append(RelativeTarget(target_id, east, north, self.plan.cruise_altitude_m))
        self.replans.append({
            **request, "attempt": self.attempts, "status": "PASS", "elapsed_s": elapsed_s,
            "new_route_generation": generation + 1,
            "start_cell": list(current), "invalidated_route": [list(cell) for cell in remaining_route],
            "raw_outbound": [list(cell) for cell in outbound],
            "compacted_outbound": [list(cell) for cell in outbound_compact],
            "raw_return": [list(cell) for cell in returning],
            "compacted_return": [list(cell) for cell in return_compact],
            "replacement_target_ids": [target.target_id for target in targets],
        })
        self.route_generation = generation + 1
        self.successful_replans += 1
        self.consecutive_failures = 0
        self.failure_reason = None
        return tuple(targets)

"""ROS-independent v0.15.1 range observation, mapping, and bounded replanning."""

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
        self.observations: list[dict[str, Any]] = []
        self.map_updates: list[dict[str, Any]] = []
        self.replans: list[dict[str, Any]] = []
        self.failure_reason: str | None = None

    def observe_pose(self, pose: PoseSample) -> None:
        if self.pose and pose.session_id == self.pose.session_id and (
            pose.source_time_boot_ms <= self.pose.source_time_boot_ms
            or pose.receipt_monotonic_ns <= self.pose.receipt_monotonic_ns
        ):
            return
        if all(isfinite(value) for value in (pose.east_m, pose.north_m, pose.heading_enu_deg)):
            self.pose = pose

    def _reject(self, observation: RangeObservation, reason: str) -> None:
        self.observations.append({
            "sequence": observation.sequence, "session_id": observation.session_id,
            "sensor_time_ns": observation.sensor_time_ns, "accepted": False, "reason": reason,
        })

    def process(
        self, observation: RangeObservation, *, launch_east_m: float, launch_north_m: float,
        remaining_route: tuple[Cell, ...], now_ns: int,
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
            point = (
                relative_east + measured_range * cos(angle),
                relative_north + measured_range * sin(angle),
            )
            try:
                cell = self.plan.transform.enu_to_cell(point)
            except ValueError:
                continue
            if cell not in self.occupied:
                discovered.add(cell)
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
        intersection = tuple(cell for cell in remaining_route if cell in updated)
        self.map_updates.append({
            "sequence": observation.sequence,
            "new_occupied_cells": [list(cell) for cell in sorted(discovered)],
            "new_inflated_cells": [list(cell) for cell in sorted(new_inflated)],
            "remaining_route_intersection": [list(cell) for cell in intersection],
        })
        if not intersection:
            return None
        if self.attempts >= self.limits.maximum_attempts:
            self.failure_reason = "bounded replanning attempt limit exceeded"
            return ()
        self.attempts += 1
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
            return ()
        if outbound is None or returning is None:
            self.failure_reason = "no safe route after sensor discovery"
            self.replans.append({"attempt": self.attempts, "status": "FAIL", "reason": self.failure_reason})
            return ()
        outbound_compact = compact_path(outbound, updated)
        return_compact = compact_path(returning, updated)
        cells = outbound_compact[1:] + return_compact[1:]
        targets: list[RelativeTarget] = []
        for index, cell in enumerate(cells):
            east, north = self.plan.transform.cell_center_to_enu(cell)
            outbound_count = len(outbound_compact) - 1
            if index < outbound_count:
                target_id = "replan-outbound-goal" if index == outbound_count - 1 else f"replan-outbound-{index + 1:03d}"
            else:
                return_index = index - outbound_count
                target_id = "replan-return-launch" if return_index == len(return_compact) - 2 else f"replan-return-{return_index + 1:03d}"
            targets.append(RelativeTarget(target_id, east, north, self.plan.cruise_altitude_m))
        self.replans.append({
            "attempt": self.attempts, "status": "PASS", "elapsed_s": elapsed_s,
            "start_cell": list(current), "invalidated_route": [list(cell) for cell in remaining_route],
            "raw_outbound": [list(cell) for cell in outbound],
            "compacted_outbound": [list(cell) for cell in outbound_compact],
            "raw_return": [list(cell) for cell in returning],
            "compacted_return": [list(cell) for cell in return_compact],
            "replacement_target_ids": [target.target_id for target in targets],
        })
        return tuple(targets)

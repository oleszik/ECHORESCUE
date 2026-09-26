"""Simulation-only v0.16.0 autonomous LiDAR frontier exploration node."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from time import monotonic, monotonic_ns
from typing import Any

import rclpy

from echorescue.frontier_exploration import (
    ExplorationState, FrontierCandidate, FrontierOccupancyMap, FrontierPolicy,
    MappingPolicy,
)
from echorescue.planner_flight import Cell, GridTransform, compact_path
from echorescue.sensor_replanning import PoseSample, RangeObservation
from echorescue.waypoint_mission import (
    NavigationPhase, RelativeTarget, WaypointMissionConfig,
    WaypointMissionController, serialize_waypoint_report,
)
from echorescue_interfaces.msg import (
    EchoRescueVehicleState3D, ExplorationState as ExplorationStateMessage,
    RangeObstacleObservation,
)
from echorescue_ros.mavlink_waypoint_mission import MavlinkWaypointMission
from echorescue_ros.qos import MISSION_QOS, TELEMETRY_QOS


class MavlinkFrontierExploration(MavlinkWaypointMission):
    """Bind controller-owned mapping/frontiers to the existing safe executor."""

    def __init__(self, output_path: Path, config_path: Path) -> None:
        self.raw: dict[str, Any] = json.loads(config_path.read_text(encoding="utf-8"))
        if self.raw.get("schema_version") != "echorescue-frontier-exploration/1.0":
            raise ValueError("unsupported frontier exploration configuration")
        super().__init__(output_path)
        bounds, sensor, policy, flight = (
            self.raw["bounds"], self.raw["sensor"], self.raw["frontier_policy"],
            self.raw["flight"],
        )
        transform = GridTransform(
            int(bounds["rows"]), int(bounds["columns"]), float(bounds["resolution_m"]),
            float(bounds["origin_relative_to_launch_enu_m"][0]),
            float(bounds["origin_relative_to_launch_enu_m"][1]),
            "east_positive", "north_positive",
        )
        self.map = FrontierOccupancyMap(MappingPolicy(
            transform=transform,
            footprint_m=float(policy["conservative_vehicle_radius_m"]) + float(policy["safety_margin_m"]),
            sensor_frame=str(sensor["sensor_frame"]),
            sensor_yaw_offset_enu_deg=float(sensor["sensor_yaw_offset_enu_deg"]),
            range_min_m=float(sensor["range_min_m"]), range_max_m=float(sensor["range_max_m"]),
            horizontal_field_of_view_rad=float(sensor["horizontal_field_of_view_rad"]),
            observation_max_age_s=float(sensor["observation_max_age_s"]),
            pose_max_age_s=float(sensor["pose_max_age_s"]),
            pose_observation_max_skew_s=float(sensor["pose_observation_max_skew_s"]),
            maximum_planar_tilt_rad=float(sensor["maximum_planar_tilt_rad"]),
            maximum_map_revisions=int(policy["maximum_map_revisions"]),
        ))
        self.frontier_policy = FrontierPolicy(
            minimum_cluster_size=int(policy["minimum_cluster_size"]),
            information_radius_cells=int(policy["information_radius_cells"]),
            information_gain_weight=float(policy["information_gain_weight"]),
            path_cost_weight=float(policy["path_cost_weight"]),
            target_switch_penalty=float(policy["target_switch_penalty"]),
        )
        config = WaypointMissionConfig(
            targets=(RelativeTarget("g001-launch-stabilize", 0, 0, float(flight["takeoff_altitude_m"])),),
            takeoff_altitude_m=float(flight["takeoff_altitude_m"]),
            horizontal_tolerance_m=float(flight["horizontal_tolerance_m"]),
            vertical_tolerance_m=float(flight["vertical_tolerance_m"]),
            settling_time_s=float(flight["settling_time_s"]),
            target_timeout_s=float(flight["target_timeout_s"]),
            progress_timeout_s=float(flight["progress_timeout_s"]),
            progress_epsilon_m=float(flight["progress_epsilon_m"]),
            mission_timeout_s=float(flight["mission_timeout_s"]),
            geofence_horizontal_radius_m=float(flight["geofence_horizontal_radius_m"]),
            geofence_min_altitude_m=float(flight["geofence_min_altitude_m"]),
            geofence_max_altitude_m=float(flight["geofence_max_altitude_m"]),
            ready_timeout_s=float(flight["initialization_timeout_s"]),
            preflight_hold_s=float(flight["preflight_hold_s"]),
            transition_timeout_s=float(flight["transition_timeout_s"]),
            takeoff_timeout_s=float(flight["takeoff_timeout_s"]),
            landing_timeout_s=float(flight["landing_timeout_s"]),
            cleanup_timeout_s=float(flight["cleanup_timeout_s"]),
            dynamic_targets=True,
            dynamic_target_wait_timeout_s=float(policy["selection_timeout_s"]),
        )
        self.controller = WaypointMissionController(config, monotonic_ns())
        self.state = ExplorationState.INITIALIZING
        self.route_generation = 1
        self.frontier_target_count = 0
        self.replan_count = 0
        self.active_frontier_replans = 0
        self.consecutive_unreachable = 0
        self.visited_frontiers: set[Cell] = set()
        self.active_route: tuple[Cell, ...] = ()
        self.active_frontier: Cell | None = None
        self.returning = False
        self.exploration_failure: str | None = None
        self.completion_reason: str | None = None
        self.last_map_change_ns = monotonic_ns()
        self.last_scan_ns = 0
        self.last_selection_ns = 0
        self._exploration_session_id = ""
        self.frontier_history: list[dict[str, Any]] = []
        self.route_history: list[dict[str, Any]] = []
        self.invalidations: list[dict[str, Any]] = []
        self._exploration_sequence = 0
        self._map_event_revision = 0
        self._state_pub = self.create_publisher(
            ExplorationStateMessage, "/echorescue/exploration/state", MISSION_QOS,
        )
        self.create_subscription(
            EchoRescueVehicleState3D, "/echorescue/vehicle/state_3d", self._pose,
            TELEMETRY_QOS,
        )
        self.create_subscription(
            RangeObstacleObservation, "/echorescue/sensors/range_observation",
            self._scan, MISSION_QOS,
        )
        self._publish_exploration("mission_started", "waiting for telemetry and LiDAR")

    def _pose(self, message: EchoRescueVehicleState3D) -> None:
        if not message.position_valid or not message.has_heading or not message.has_attitude:
            return
        self.map.observe_pose(PoseSample(
            message.session_id, int(message.source_time_boot_ms), monotonic_ns(),
            float(message.x_m), float(message.y_m), float(message.heading_deg),
            float(message.roll_rad), float(message.pitch_rad),
        ))

    def _scan(self, message: RangeObstacleObservation) -> None:
        launch = self.controller.launch_enu
        if launch is None:
            return
        before = self.map.map_revision
        accepted_before = self.map.accepted_scans
        try:
            changed = self.map.process(
                RangeObservation(
                    message.session_id, int(message.sequence), int(message.sensor_time_ns),
                    int(message.receipt_monotonic_ns), message.sensor_frame,
                    float(message.angle_min_rad), float(message.angle_increment_rad),
                    float(message.range_min_m), float(message.range_max_m),
                    tuple(float(value) for value in message.ranges_m),
                ),
                launch_east_m=launch[0], launch_north_m=launch[1], now_ns=monotonic_ns(),
            )
        except RuntimeError as error:
            self._fail(str(error))
            return
        if self.map.accepted_scans > accepted_before:
            self.last_scan_ns = monotonic_ns()
        if changed:
            self.last_map_change_ns = monotonic_ns()
            self.controller.record_external_event(
                self.last_map_change_ns, "occupancy_map_updated",
                f"controller-owned map revision {self.map.map_revision}",
            )
            self._publish_exploration("map_revision", f"revision {self.map.map_revision}")
            if before != self.map.map_revision:
                self._invalidate_unsafe_route()

    def _current_cell(self) -> Cell | None:
        pose, launch = self.map.pose, self.controller.launch_enu
        if pose is None or launch is None or pose.session_id != self.controller.session_id:
            return None
        try:
            return self.map.policy.transform.enu_to_cell((pose.east_m - launch[0], pose.north_m - launch[1]))
        except ValueError:
            return None

    def _path_targets(self, path: tuple[Cell, ...], purpose: str) -> tuple[RelativeTarget, ...]:
        blocked = frozenset(
            (row, column)
            for row in range(self.map.policy.transform.rows)
            for column in range(self.map.policy.transform.columns)
            if not self.map.safe_free((row, column))
        )
        compacted = compact_path(path, blocked)
        targets: list[RelativeTarget] = []
        altitude = float(self.raw["flight"]["takeoff_altitude_m"])
        for index, cell in enumerate(compacted[1:], 1):
            east, north = self.map.policy.transform.cell_center_to_enu(cell)
            targets.append(RelativeTarget(
                f"g{self.route_generation:03d}-{purpose}-{index:03d}", east, north, altitude,
            ))
        return tuple(targets)

    def _command_horizon(self, path: tuple[Cell, ...]) -> tuple[Cell, ...]:
        """Limit each generation to a bounded distance, independent of map resolution.

        A shorter receding horizon means new occupancy inflation is detected
        and replanned against sooner, before the vehicle can be commanded deep
        into a corridor that a later scan reveals as unsafe. The distance is
        configurable per mission profile. Legacy profiles retain the 1 m
        behavior; newer profiles can opt into a shorter horizon.
        """
        horizon_m = float(self.raw["flight"].get("command_horizon_m", 1.0))
        cell_steps = max(1, int(horizon_m / self.map.policy.transform.resolution_m))
        return path[:cell_steps + 1]

    def _route_budget_available(self) -> bool:
        return self.route_generation < int(self.raw["limits"]["maximum_route_generations"])

    def _install(self, path: tuple[Cell, ...], purpose: str, now_ns: int) -> bool:
        if not self._route_budget_available():
            self._fail("route generation budget exhausted")
            return False
        command_path = self._command_horizon(path)
        targets = self._path_targets(command_path, purpose)
        if not targets:
            return False
        self.active_route = command_path
        self.route_history.append({
            "route_generation": self.route_generation, "purpose": purpose,
            "planned_monotonic_ns": now_ns, "path": [list(cell) for cell in path],
            "commanded_horizon": [list(cell) for cell in command_path],
            "target_ids": [target.target_id for target in targets],
            "known_safe_only": all(self.map.safe_free(cell) for cell in path),
            "map_revision": self.map.map_revision,
        })
        self.controller.install_dynamic_targets(targets, now_ns)
        self.controller.record_external_event(now_ns, "route_replanned", f"generation {self.route_generation} {purpose}")
        self.last_selection_ns = now_ns
        self._publish_exploration("route_selected", purpose)
        return True

    def _select(self, now_ns: int) -> None:
        current = self._current_cell()
        if current is None or not self.map.safe_free(current):
            self._fail("no safe current cell for frontier selection")
            return
        if self.returning:
            launch_cell = self.map.policy.transform.enu_to_cell((0.0, 0.0))
            path = self.map.path(current, launch_cell)
            if path is None:
                self._fail("no safe return route in discovered map")
            elif len(path) == 1:
                self.controller.complete_dynamic_mission(now_ns, self.completion_reason or self.exploration_failure or "return complete")
                self.state = ExplorationState.LANDING
                self._publish_exploration("land_requested", "return target settled")
            else:
                self.route_generation += 1
                self._install(path, "return", now_ns)
                self.state = ExplorationState.RETURNING
            return
        if self.active_frontier is not None and self.active_frontier not in self.visited_frontiers:
            active_path = self.map.path(current, self.active_frontier)
            if active_path is not None and len(active_path) > 1:
                self.route_generation += 1
                if self._install(active_path, "frontier", now_ns):
                    self.state = ExplorationState.NAVIGATING
                return
            if active_path is not None:
                self.visited_frontiers.add(self.active_frontier)
                self.controller.record_external_event(
                    now_ns, "frontier_settled", f"frontier cell {self.active_frontier} reached",
                )
                self._publish_exploration("frontier_settled", "selected frontier reached")
                self.active_frontier = None
                self.last_selection_ns = now_ns
                return
            # A newly unsafe/unreachable target is suppressed once; subsequent
            # selection may choose another reachable cluster deterministically.
            self.visited_frontiers.add(self.active_frontier)
            self.active_frontier = None
            self.consecutive_unreachable += 1
            if self.consecutive_unreachable > int(
                self.raw["limits"]["maximum_consecutive_unreachable_frontiers"]
            ):
                self._fail("consecutive unreachable frontier budget exhausted")
                return
        candidates = tuple(
            item for item in self.map.candidates(current, self.frontier_policy, self.active_frontier)
            if item.cell not in self.visited_frontiers
        )
        self.frontier_history.append({
            "map_revision": self.map.map_revision, "selected_monotonic_ns": now_ns,
            "clusters": [candidate.report() for candidate in candidates],
        })
        candidate = candidates[0] if candidates else None
        if candidate is None:
            quiet_s = (now_ns - self.last_map_change_ns) / 1e9
            if quiet_s < float(self.raw["completion"]["map_quiet_s"]):
                return
            ratio = self.map.known_ratio()
            if ratio < float(self.raw["completion"]["minimum_known_ratio"]):
                self._fail("no reachable frontier before minimum known-space ratio")
                return
            self.completion_reason = "no reachable safe frontier after map-settling interval"
            self.state = ExplorationState.EXPLORATION_COMPLETE
            self.returning = True
            self.controller.record_external_event(now_ns, "exploration_complete", self.completion_reason)
            self._publish_exploration("exploration_complete", self.completion_reason)
            self._select(now_ns)
            return
        if self.frontier_target_count >= int(self.raw["limits"]["maximum_frontier_targets"]):
            self.exploration_failure = "frontier target budget exhausted"
            self.returning = True
            self.state = ExplorationState.RETURNING
            self._publish_exploration("budget_exhausted", self.exploration_failure)
            self._select(now_ns)
            return
        self.frontier_target_count += 1
        self.consecutive_unreachable = 0
        self.active_frontier = candidate.cell
        self.active_frontier_replans = 0
        self.state = ExplorationState.SELECTING_FRONTIER
        self.frontier_history[-1]["selected"] = candidate.report()
        self._publish_exploration("frontier_selected", "deterministic frontier score selected", candidate)
        if len(candidate.path) == 1:
            # A frontier is visited only after telemetry places the vehicle in its
            # cell.  Marking it at selection time suppresses a distant cluster
            # when only a short receding-horizon prefix is commanded.
            self.visited_frontiers.add(candidate.cell)
            self.active_frontier = None
            self.controller.record_external_event(
                now_ns, "frontier_settled", f"frontier cell {candidate.cell} reached",
            )
            self.last_selection_ns = now_ns
            self._publish_exploration("frontier_settled", "selected frontier reached", candidate)
            return
        self.route_generation += 1
        if self._install(candidate.path, "frontier", now_ns):
            self.state = ExplorationState.NAVIGATING
        else:
            self.consecutive_unreachable += 1
            self._publish_exploration("frontier_unreachable", "candidate produced no executable path")

    def _invalidate_unsafe_route(self) -> None:
        if self.controller.phase not in {
            NavigationPhase.SEND_TARGET, NavigationPhase.WAIT_TRANSMISSION,
            NavigationPhase.TRACK_TARGET,
        } or not any(cell in self.map.inflated for cell in self.active_route):
            return
        now_ns = monotonic_ns()
        self.invalidations.append({
            "route_generation": self.route_generation, "map_revision": self.map.map_revision,
            "reason": "new occupied inflation intersects active route",
        })
        if self.replan_count >= int(self.raw["limits"]["maximum_replans_per_mission"]):
            self._fail("replan budget exhausted")
            return
        if self.returning:
            current = self._current_cell()
            launch_cell = self.map.policy.transform.enu_to_cell((0.0, 0.0))
            path = None if current is None else self.map.path(current, launch_cell)
            if path is None or len(path) < 2:
                self._fail("no safe return route in discovered map")
                return
            self.replan_count += 1
            self.route_generation += 1
            if not self._route_budget_available():
                self._fail("route generation budget exhausted")
                return
            command_path = self._command_horizon(path)
            targets = self._path_targets(command_path, "return")
            self.controller.replace_remaining_targets(
                targets, now_ns, "new occupancy invalidated conservative return",
            )
            self.active_route = command_path
            self.route_history.append({
                "route_generation": self.route_generation, "purpose": "return_replan",
                "map_revision": self.map.map_revision, "path": [list(cell) for cell in path],
                "commanded_horizon": [list(cell) for cell in command_path],
                "target_ids": [target.target_id for target in targets],
                "known_safe_only": True,
            })
            self._publish_exploration("route_invalidated", "conservative return replanned")
            return
        if self.active_frontier_replans >= int(
            self.raw["limits"]["maximum_replans_per_target"]
        ):
            self._fail("active-frontier replan budget exhausted")
            return
        current = self._current_cell()
        path = None if current is None or self.active_frontier is None else self.map.path(current, self.active_frontier)
        replacement_frontier = self.active_frontier
        if path is None or len(path) < 2:
            if self.active_frontier is not None:
                self.visited_frontiers.add(self.active_frontier)
            alternatives = () if current is None else tuple(
                item for item in self.map.candidates(current, self.frontier_policy, self.active_frontier)
                if item.cell not in self.visited_frontiers
            )
            if not alternatives:
                assert current is not None
                if not self.map.safe_free(current):
                    # The current cell itself is not known-safe (for example,
                    # new occupancy inflation has just swept over it). Neither
                    # a hold nor a return commitment may be issued from here;
                    # defer to the existing failure/recovery policy instead of
                    # commanding a target that would violate the known-safe
                    # route contract.
                    self._fail("no safe current cell to hold during frontier replan")
                    return
                launch_cell = self.map.policy.transform.enu_to_cell((0.0, 0.0))
                return_path = self.map.path(current, launch_cell)
                if return_path is not None:
                    # A known-safe route home already exists: abort exploration
                    # and commit to that return rather than holding indefinitely
                    # on a cell that will never yield a new reachable frontier.
                    self.replan_count += 1
                    self.route_generation += 1
                    if not self._route_budget_available():
                        self._fail("route generation budget exhausted")
                        return
                    self.active_frontier = None
                    self.returning = True
                    self.completion_reason = (
                        self.completion_reason
                        or "active frontier became unreachable; returning on known-safe route"
                    )
                    if len(return_path) == 1:
                        self.active_route = return_path
                        self.controller.complete_dynamic_mission(now_ns, self.completion_reason)
                        self.state = ExplorationState.LANDING
                        self._publish_exploration(
                            "land_requested",
                            "already at launch cell; landing after frontier abort",
                        )
                        return
                    command_path = self._command_horizon(return_path)
                    targets = self._path_targets(command_path, "return")
                    if not targets:
                        self._fail("active frontier became unreachable")
                        return
                    self.controller.replace_remaining_targets(
                        targets, now_ns,
                        "active frontier unreachable; committing to known-safe return",
                    )
                    self.active_route = command_path
                    self.route_history.append({
                        "route_generation": self.route_generation, "purpose": "return",
                        "map_revision": self.map.map_revision,
                        "path": [list(cell) for cell in return_path],
                        "commanded_horizon": [list(cell) for cell in command_path],
                        "target_ids": [target.target_id for target in targets],
                        "known_safe_only": True,
                    })
                    self.state = ExplorationState.RETURNING
                    self._publish_exploration(
                        "returning",
                        "active frontier unreachable; known-safe return commanded",
                    )
                    return
                self.replan_count += 1
                self.active_frontier_replans += 1
                self.route_generation += 1
                east, north = self.map.policy.transform.cell_center_to_enu(current)
                hold = RelativeTarget(
                    f"g{self.route_generation:03d}-replan-hold-001", east, north,
                    float(self.raw["flight"]["takeoff_altitude_m"]),
                )
                self.controller.replace_remaining_targets(
                    (hold,), now_ns,
                    "unsafe frontier suppressed; holding in known-safe current cell",
                )
                self.active_frontier = None
                self.active_route = (current,)
                self.route_history.append({
                    "route_generation": self.route_generation,
                    "purpose": "replan_hold", "map_revision": self.map.map_revision,
                    "path": [list(current)], "commanded_horizon": [list(current)],
                    "target_ids": [hold.target_id], "known_safe_only": True,
                })
                self.state = ExplorationState.REPLANNING
                self._publish_exploration(
                    "frontier_unreachable",
                    "unsafe frontier suppressed; safe hold installed before reselection",
                )
                return
            alternative = alternatives[0]
            path = alternative.path
            replacement_frontier = alternative.cell
            self.frontier_history.append({
                "map_revision": self.map.map_revision,
                "selected_monotonic_ns": now_ns,
                "reason": "unsafe active frontier suppressed",
                "clusters": [item.report() for item in alternatives],
                "selected": alternative.report(),
            })
        self.replan_count += 1
        self.active_frontier_replans += 1
        self.route_generation += 1
        self.active_frontier = replacement_frontier
        if not self._route_budget_available():
            self._fail("route generation budget exhausted")
            return
        command_path = self._command_horizon(path)
        targets = self._path_targets(command_path, "replan")
        if not targets:
            self._fail("active frontier became unreachable")
            return
        self.controller.replace_remaining_targets(targets, now_ns, "new occupancy invalidated active route")
        self.active_route = command_path
        self.route_history.append({
            "route_generation": self.route_generation, "purpose": "replan",
            "map_revision": self.map.map_revision, "path": [list(cell) for cell in path],
            "commanded_horizon": [list(cell) for cell in command_path],
            "target_ids": [target.target_id for target in targets], "known_safe_only": True,
        })
        self._publish_exploration("route_invalidated", "replacement installed")

    def _fail(self, reason: str) -> None:
        if self.controller.terminal or self.exploration_failure is not None:
            return
        self.exploration_failure = reason
        self.state = ExplorationState.RECOVERY
        self.controller.record_external_event(monotonic_ns(), "exploration_failed", reason)
        self.controller.cancel(monotonic_ns(), reason)
        self._publish_exploration("recovery", reason)

    def _publish_exploration(self, event: str, reason: str,
                             candidate: FrontierCandidate | None = None) -> None:
        self._exploration_sequence += 1
        counts = self.map.counts()
        message = ExplorationStateMessage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.schema_version = "echorescue-exploration-state/1.0"
        message.session_id = self.controller.session_id
        message.sequence = self._exploration_sequence
        message.monotonic_ns = monotonic_ns()
        message.state = self.state.value
        message.event = event
        message.reason = reason
        message.map_revision = self.map.map_revision
        message.unknown_cell_count = counts["unknown_cell_count"]
        message.free_cell_count = counts["free_cell_count"]
        message.occupied_cell_count = counts["occupied_cell_count"]
        message.inflated_cell_count = counts["inflated_cell_count"]
        message.accepted_scan_count = self.map.accepted_scans
        message.rejected_scan_count = self.map.rejected_scans
        message.accepted_beam_count = self.map.accepted_beams
        message.rejected_beam_count = self.map.rejected_beams
        candidates = self.map.frontier_clusters(self.frontier_policy.minimum_cluster_size)
        message.frontier_cluster_count = len(candidates)
        selected = candidate.cell if candidate else self.active_frontier
        message.has_selected_frontier = selected is not None
        message.selected_frontier_row = -1 if selected is None else selected[0]
        message.selected_frontier_column = -1 if selected is None else selected[1]
        message.frontier_score = 0.0 if candidate is None else candidate.score
        message.frontier_path_cost = 0 if candidate is None else candidate.path_cost
        message.frontier_information_gain = 0 if candidate is None else candidate.information_gain
        message.route_generation = self.route_generation
        message.active_target_id = self.controller.active_target_id
        self._state_pub.publish(message)

    def _advance_mission(self) -> None:
        now_ns = monotonic_ns()
        session_id = self.controller.session_id
        if session_id and self._exploration_session_id and session_id != self._exploration_session_id:
            self.exploration_failure = "MAVLink session changed during exploration"
            self.active_route = ()
            self.active_frontier = None
            self.state = ExplorationState.RECOVERY
            self.controller.record_external_event(
                now_ns, "exploration_session_invalidated",
                "old-session map/pose, target and route correlation invalidated",
            )
            self._publish_exploration("recovery", self.exploration_failure)
        if session_id:
            self._exploration_session_id = session_id
        if self.controller.launch_enu is not None and self.map.accepted_scans == 0:
            self.state = ExplorationState.MAPPING
            return
        if self.exploration_failure is None and self.controller.flight.armed and self.last_scan_ns and (
            now_ns - self.last_scan_ns
        ) / 1e9 > float(self.raw["sensor"]["dropout_timeout_s"]):
            self._fail("LiDAR telemetry became stale while airborne")
        elapsed_s = (now_ns - self.controller.started_monotonic_ns) / 1e9
        if (
            not self.returning
            and self.controller.flight.armed
            and elapsed_s >= float(self.raw["limits"]["latest_forced_return_s"])
        ):
            self.exploration_failure = "latest forced-return time reached"
            self.returning = True
            self.state = ExplorationState.RETURNING
            self.controller.record_external_event(now_ns, "forced_return", self.exploration_failure)
            if self.controller.phase is NavigationPhase.WAIT_TARGETS:
                self._select(now_ns)
        if self.controller.phase is NavigationPhase.WAIT_TARGETS:
            minimum_interval = float(self.raw["limits"]["minimum_target_switch_interval_s"])
            if (now_ns - self.last_selection_ns) / 1e9 >= minimum_interval:
                self._select(now_ns)
        super()._advance_mission()
        if self.controller.terminal:
            self.state = ExplorationState.SUCCEEDED if self.controller.succeeded and self.exploration_failure is None else ExplorationState.FAILED

    def _write_report(self) -> None:
        if self._report_written:
            return
        self.state = ExplorationState.SUCCEEDED if self.controller.succeeded and self.exploration_failure is None else ExplorationState.FAILED
        report = self.controller.report()
        compact_revisions = [
            {
                **{key: value for key, value in revision.items()
                   if key not in {"new_free_cells", "new_occupied_cells"}},
                "new_free_cell_count": len(revision["new_free_cells"]),
                "new_occupied_cell_count": len(revision["new_occupied_cells"]),
            }
            for revision in self.map.revisions
        ]

        def compact_candidate(candidate: dict[str, Any]) -> dict[str, Any]:
            return {
                key: value for key, value in candidate.items()
                if key not in {"cluster", "path"}
            } | {
                "cluster_size": len(candidate.get("cluster", [])),
            }

        compact_frontiers: list[dict[str, Any]] = []
        for item in self.frontier_history:
            compact = dict(item)
            compact["clusters"] = [compact_candidate(value) for value in item.get("clusters", [])]
            if "selected" in item:
                compact["selected"] = compact_candidate(item["selected"])
            compact_frontiers.append(compact)
        report.update({
            "schema_version": "echorescue-frontier-exploration-mission/1.0",
            "milestone": "v0.16.0", "exploration_state": self.state.value,
            "configuration_digest": hashlib.sha256(json.dumps(self.raw, sort_keys=True).encode()).hexdigest(),
            "controller_prior_interior_geometry": False,
            "map_transform": asdict(self.map.policy.transform),
            "map_revisions": compact_revisions, "map_counts": self.map.counts(),
            "occupied_cells": [list(cell) for cell in sorted(self.map.occupied)],
            "inflated_cells": [list(cell) for cell in sorted(self.map.inflated)],
            "accepted_scan_count": self.map.accepted_scans,
            "rejected_scan_count": self.map.rejected_scans,
            "accepted_beam_count": self.map.accepted_beams,
            "rejected_beam_count": self.map.rejected_beams,
            "sensor_rejections": self.map.rejections,
            "frontier_history": compact_frontiers,
            "route_history": self.route_history, "route_generation": self.route_generation,
            "route_invalidations": self.invalidations, "replan_count": self.replan_count,
            "active_frontier_replan_count": self.active_frontier_replans,
            "frontier_target_count": self.frontier_target_count,
            "exploration_completion_reason": self.completion_reason,
            "exploration_failure_reason": self.exploration_failure,
            "controller_known_ratio": self.map.known_ratio(),
            "autopilot_status_text": list(self._autopilot_status_text),
        })
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(serialize_waypoint_report(report), encoding="utf-8")
        self.passed = self.controller.succeeded and self.exploration_failure is None
        self.finished = True
        self._report_written = True


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = MavlinkFrontierExploration(parsed.output, parsed.config)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.cancel_for_shutdown()
        deadline = monotonic() + node.controller.config.cleanup_timeout_s + 1
        while rclpy.ok() and not node.finished and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if not node.finished and node.controller.terminal:
            node._write_report()
        passed = node.passed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()

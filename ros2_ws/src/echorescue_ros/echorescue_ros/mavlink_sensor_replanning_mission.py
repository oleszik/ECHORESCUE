"""Simulation-only v0.15.1 sensor-driven bounded replanning mission."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import monotonic_ns

import rclpy

from echorescue.planner_flight import KnownMapPlan, plan_known_map
from echorescue.sensor_replanning import PoseSample, RangeObservation, ReplanningLimits, SensorMapReplanner
from echorescue.waypoint_mission import NavigationPhase
from echorescue_interfaces.msg import EchoRescueVehicleState3D, RangeObstacleObservation
from echorescue_ros.mavlink_waypoint_mission import MavlinkWaypointMission
from echorescue_ros.qos import MISSION_QOS, TELEMETRY_QOS


class MavlinkSensorReplanningMission(MavlinkWaypointMission):
    def __init__(self, output_path: Path, planner_config: Path) -> None:
        super().__init__(output_path)
        raw = json.loads(planner_config.read_text(encoding="utf-8"))
        self.plan = plan_known_map(raw)
        sensor = raw["sensor_replanning"]
        self.replanner = SensorMapReplanner(self.plan, ReplanningLimits(
            observation_max_age_s=float(sensor["observation_max_age_s"]),
            pose_max_age_s=float(sensor["pose_max_age_s"]),
            pose_observation_max_skew_s=float(sensor["pose_observation_max_skew_s"]),
            maximum_attempts=int(sensor["maximum_replan_attempts"]),
            timeout_s=float(sensor["replan_timeout_s"]),
            sensor_yaw_offset_enu_deg=float(sensor["sensor_yaw_offset_enu_deg"]),
            sensor_frame=str(sensor["sensor_frame"]),
            range_min_m=float(sensor["range_min_m"]),
            range_max_m=float(sensor["range_max_m"]),
            horizontal_field_of_view_rad=float(sensor["horizontal_field_of_view_rad"]),
            maximum_replans_per_target=int(sensor.get("maximum_replans_per_target", 1)),
            cooldown_s=float(sensor.get("replan_cooldown_s", 0.5)),
            maximum_consecutive_failures=int(sensor.get("maximum_consecutive_planning_failures", 1)),
            map_change_threshold_cells=int(sensor.get("map_change_threshold_cells", 1)),
        ))
        self.remaining_route = self.plan.raw_outbound + self.plan.raw_return
        self.create_subscription(EchoRescueVehicleState3D, "/echorescue/vehicle/state_3d", self._pose, TELEMETRY_QOS)
        self.create_subscription(RangeObstacleObservation, "/echorescue/sensors/range_observation", self._scan, MISSION_QOS)

    def _pose(self, message: EchoRescueVehicleState3D) -> None:
        if not message.position_valid or not message.has_heading:
            return
        self.replanner.observe_pose(PoseSample(
            message.session_id, int(message.source_time_boot_ms), int(message.receipt_monotonic_ns),
            float(message.x_m), float(message.y_m), float(message.heading_deg),
        ))

    def _scan(self, message: RangeObstacleObservation) -> None:
        launch = self.controller.launch_enu
        pose = self.replanner.pose
        if (launch is None or pose is None
                or self.controller.phase not in {
                    NavigationPhase.SEND_TARGET, NavigationPhase.WAIT_TRANSMISSION,
                    NavigationPhase.TRACK_TARGET,
                }):
            return
        try:
            current = self.plan.transform.enu_to_cell((pose.east_m - launch[0], pose.north_m - launch[1]))
        except ValueError:
            return
        remaining = _remaining_from(current, self.remaining_route)
        observation_count = len(self.replanner.observations)
        update_count = len(self.replanner.map_updates)
        skipped_count = len(self.replanner.skipped_replans)
        replan_count = len(self.replanner.replans)
        replacement = self.replanner.process(
            RangeObservation(
                message.session_id, int(message.sequence), int(message.sensor_time_ns),
                int(message.receipt_monotonic_ns), message.sensor_frame,
                float(message.angle_min_rad), float(message.angle_increment_rad),
                float(message.range_min_m), float(message.range_max_m), tuple(float(value) for value in message.ranges_m),
            ),
            launch_east_m=launch[0], launch_north_m=launch[1],
            remaining_route=remaining, now_ns=monotonic_ns(),
            target_id=self.controller.active_target_id,
            route_generation=self.replanner.route_generation,
        )
        latest = self.replanner.observations[-1] if len(self.replanner.observations) > observation_count else {}
        if latest.get("accepted") and latest.get("discovered_cells"):
            self.controller.record_external_event(monotonic_ns(), "sensor_observation_accepted", "fresh same-session lidar scan associated with vehicle pose")
        if len(self.replanner.map_updates) > update_count:
            self.controller.record_external_event(monotonic_ns(), "occupancy_map_updated", f"map revision {self.replanner.map_revision}: sensor endpoint cells added and conservatively inflated")
        if len(self.replanner.skipped_replans) > skipped_count:
            self.controller.record_external_event(monotonic_ns(), "replan_skipped", self.replanner.skipped_replans[-1]["reason"])
        if replacement == ():
            self.controller.record_external_event(monotonic_ns(), "route_invalidated", "new inflated occupancy intersects remaining route")
            event = "replan_skipped" if len(self.replanner.replans) > replan_count and self.replanner.replans[-1]["status"] == "SKIPPED" else "replan_failed"
            self.controller.record_external_event(monotonic_ns(), event, self.replanner.failure_reason or "sensor replan failed")
            self.controller.cancel(monotonic_ns(), self.replanner.failure_reason or "sensor replan failed")
        elif replacement:
            self.controller.record_external_event(monotonic_ns(), "route_invalidated", "new inflated occupancy intersects remaining route")
            self.controller.record_external_event(monotonic_ns(), "route_replanned", f"route generation {self.replanner.route_generation} accepted by bounded deterministic A*")
            self.controller.replace_remaining_targets(
                replacement, monotonic_ns(), "sensor discovery intersected the remaining inflated route",
            )
            replan = self.replanner.replans[-1]
            self.remaining_route = tuple(tuple(cell) for cell in replan["raw_outbound"] + replan["raw_return"])

    def _write_report(self) -> None:
        if self._report_written:
            return
        report = self.controller.report()
        report.update({
            "schema_version": "echorescue-sensor-replanning-mission/1.0",
            "milestone": "v0.15.1",
            "sensor_observations": self.replanner.observations,
            "accepted_observation_count": sum(bool(item["accepted"]) for item in self.replanner.observations),
            "rejected_observation_count": sum(not bool(item["accepted"]) for item in self.replanner.observations),
            "discovery_batch_count": sum(bool(item.get("discovered_cells")) for item in self.replanner.observations),
            "map_updates": self.replanner.map_updates,
            "replans": self.replanner.replans,
            "skipped_replans": self.replanner.skipped_replans,
            "pose_rejections": self.replanner.pose_rejections,
            "map_revision": self.replanner.map_revision,
            "route_generation": self.replanner.route_generation,
            "replan_budget_consumed": self.replanner.attempts,
            "replanning_failure_reason": self.replanner.failure_reason,
            "autopilot_status_text": list(self._autopilot_status_text),
        })
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.passed = self.controller.succeeded
        self.finished = True
        self._report_written = True


def _remaining_from(current: tuple[int, int], route: tuple[tuple[int, int], ...]) -> tuple[tuple[int, int], ...]:
    try:
        closest = min(range(len(route)), key=lambda index: abs(route[index][0] - current[0]) + abs(route[index][1] - current[1]))
    except ValueError:
        return (current,)
    return (current,) + route[closest:]


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--planner-config", type=Path, required=True)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = MavlinkSensorReplanningMission(parsed.output, parsed.planner_config)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.cancel_for_shutdown()
        while not node.finished:
            node._advance_mission()
    finally:
        passed = node.passed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()

"""Independent ROS observer for the v0.15.1 sensor/replanning event chain."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.node import Node

from echorescue_interfaces.msg import (
    EchoRescueVehicleState3D, MavlinkVehicleState, RangeObstacleObservation,
    WaypointMissionEvent, WaypointTarget,
)
from echorescue_ros.qos import MISSION_QOS, TELEMETRY_QOS


class SensorReplanningObserver(Node):
    def __init__(self, output: Path, timeout_s: float, expect_success: bool) -> None:
        super().__init__("echorescue_sensor_replanning_observer")
        self.output, self.timeout_s, self.expect_success = output, timeout_s, expect_success
        self.started = monotonic()
        self.finished = self.passed = False
        self.session_id = ""
        self.session_changed = False
        self.guided = self.armed = self.land = self.landed = self.disarmed = False
        self.observation_count = 0
        self.events: list[str] = []
        self.target_ids: list[str] = []
        self.goal_observed = self.return_observed = False
        self.create_subscription(MavlinkVehicleState, "/echorescue/mavlink/vehicle_state", self._vehicle, TELEMETRY_QOS)
        self.create_subscription(EchoRescueVehicleState3D, "/echorescue/vehicle/state_3d", self._state, TELEMETRY_QOS)
        self.create_subscription(RangeObstacleObservation, "/echorescue/sensors/range_observation", self._scan, MISSION_QOS)
        self.create_subscription(WaypointMissionEvent, "/echorescue/mission/event", self._event, MISSION_QOS)
        self.create_subscription(WaypointTarget, "/echorescue/mission/active_target", self._target, MISSION_QOS)
        self.create_timer(0.1, self._evaluate)

    def _session(self, value: str) -> None:
        if self.session_id and value and value != self.session_id:
            self.session_changed = True
        if value:
            self.session_id = value

    def _vehicle(self, message: MavlinkVehicleState) -> None:
        self._session(message.session_id)
        self.guided |= message.flight_mode == "GUIDED"
        self.armed |= bool(message.armed)
        self.land |= self.armed and message.flight_mode == "LAND"
        self.disarmed |= self.armed and not message.armed

    def _state(self, message: EchoRescueVehicleState3D) -> None:
        self._session(message.session_id)
        self.landed |= self.land and message.has_landed_state and message.landed

    def _scan(self, message: RangeObstacleObservation) -> None:
        self._session(message.session_id)
        self.observation_count += 1

    def _event(self, message: WaypointMissionEvent) -> None:
        self._session(message.session_id)
        self.events.append(message.event)

    def _target(self, message: WaypointTarget) -> None:
        self._session(message.session_id)
        if message.target_id not in self.target_ids:
            self.target_ids.append(message.target_id)
        self.goal_observed |= message.target_id.endswith("outbound-goal")
        self.return_observed |= message.target_id.endswith("return-launch")

    def _evaluate(self) -> None:
        required = {"sensor_observation_accepted", "occupancy_map_updated", "route_invalidated"}
        if self.expect_success:
            required.add("route_replanned")
        event_ok = required.issubset(self.events)
        route = (self.goal_observed and self.return_observed) if self.expect_success else True
        final = self.guided and self.armed and self.land and self.landed and self.disarmed and route
        if event_ok and final and not self.session_changed:
            self._finish(True, "independent sensor, mission, target, LAND and disarm evidence complete")
        elif monotonic() - self.started > self.timeout_s:
            self._finish(False, "observer timed out")

    def _finish(self, passed: bool, detail: str) -> None:
        report = {
            "schema_version": "echorescue-sensor-replanning-observer/1.0", "milestone": "v0.15.1",
            "status": "PASS" if passed else "FAIL", "detail": detail,
            "session_id": self.session_id, "session_changed": self.session_changed,
            "range_observation_count": self.observation_count, "events": self.events,
            "target_ids": self.target_ids, "guided_observed": self.guided,
            "goal_target_observed": self.goal_observed, "return_target_observed": self.return_observed,
            "armed_observed": self.armed, "land_observed": self.land,
            "on_ground_observed": self.landed, "disarmed_observed": self.disarmed,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.passed, self.finished = passed, True


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=260)
    parser.add_argument("--expect-failure", action="store_true")
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = SensorReplanningObserver(parsed.output, parsed.timeout, not parsed.expect_failure)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if not node.finished:
            node._finish(False, "observer interrupted")
    finally:
        passed = node.passed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()

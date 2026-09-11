"""Independent ROS-state/event observer for the v0.14.4 waypoint mission."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic
from typing import Any

import rclpy
from rclpy.node import Node

from echorescue.sim_integration import serialize_report
from echorescue_interfaces.msg import (
    EchoRescueVehicleState3D,
    MavlinkTelemetryStatus,
    MavlinkVehicleState,
    WaypointMissionEvent,
    WaypointTarget,
)
from echorescue_ros.qos import MISSION_QOS, STATUS_QOS, TELEMETRY_QOS


class WaypointMissionObserver(Node):
    def __init__(
        self, output_path: Path, timeout_s: float,
        horizontal_tolerance_m: float, vertical_tolerance_m: float,
        settling_time_s: float,
    ) -> None:
        super().__init__("echorescue_waypoint_mission_observer")
        self.output_path = output_path
        self.timeout_s = timeout_s
        self.horizontal_tolerance_m = horizontal_tolerance_m
        self.vertical_tolerance_m = vertical_tolerance_m
        self.settling_time_s = settling_time_s
        self.started = monotonic()
        self.finished = False
        self.passed = False
        self.session_id = ""
        self.session_changed = False
        self.telemetry_fresh = False
        self.mission_node_observed = False
        self.guided_observed = False
        self.armed_observed = False
        self.land_mode_observed = False
        self.landed_after_flight = False
        self.disarmed_after_flight = False
        self.last_source_time_ms: int | None = None
        self.minimum_altitude_m: float | None = None
        self.maximum_altitude_m: float | None = None
        self.targets: list[dict[str, Any]] = []
        self.target_ids: list[str] = []
        self.settled_event_ids: list[str] = []
        self.event_sequence: list[dict[str, Any]] = []
        self.create_subscription(MavlinkVehicleState, "/echorescue/mavlink/vehicle_state", self._vehicle, TELEMETRY_QOS)
        self.create_subscription(EchoRescueVehicleState3D, "/echorescue/vehicle/state_3d", self._state, TELEMETRY_QOS)
        self.create_subscription(MavlinkTelemetryStatus, "/echorescue/mavlink/status", self._status, STATUS_QOS)
        self.create_subscription(WaypointTarget, "/echorescue/mission/active_target", self._target, MISSION_QOS)
        self.create_subscription(WaypointMissionEvent, "/echorescue/mission/event", self._event, MISSION_QOS)
        self.create_timer(0.1, self._evaluate)

    def _track_session(self, session_id: str) -> None:
        if self.session_id and session_id and session_id != self.session_id:
            self.session_changed = True
        if session_id:
            self.session_id = session_id

    def _vehicle(self, message: MavlinkVehicleState) -> None:
        self._track_session(message.session_id)
        if message.flight_mode == "GUIDED":
            self.guided_observed = True
        if message.armed:
            self.armed_observed = True
        elif self.armed_observed:
            self.disarmed_after_flight = True
        if message.flight_mode == "LAND" and self.armed_observed:
            self.land_mode_observed = True

    def _target(self, message: WaypointTarget) -> None:
        self._track_session(message.session_id)
        if message.target_id in self.target_ids:
            return
        self.target_ids.append(message.target_id)
        self.targets.append({
            "target_id": message.target_id,
            "session_id": message.session_id,
            "commanded_enu": {
                "east_m": float(message.east_m), "north_m": float(message.north_m), "up_m": float(message.up_m),
            },
            "transmitted_ned": {
                "north_m": float(message.ned_north_m), "east_m": float(message.ned_east_m), "down_m": float(message.ned_down_m),
                "yaw_rad": float(message.ned_yaw_rad), "type_mask": int(message.type_mask),
            },
            "transmit_monotonic_ns": int(message.transmit_monotonic_ns),
            "transmit_source_time_boot_ms": int(message.transmit_source_time_boot_ms),
            "first_post_command_monotonic_ns": None,
            "arrival_started_ns": None,
            "settled": False,
        })

    def _event(self, message: WaypointMissionEvent) -> None:
        self._track_session(message.session_id)
        self.event_sequence.append({
            "sequence": int(message.sequence), "event": message.event,
            "target_id": message.target_id, "session_id": message.session_id,
        })
        if message.event == "target_settled" and message.target_id not in self.settled_event_ids:
            self.settled_event_ids.append(message.target_id)

    def _state(self, message: EchoRescueVehicleState3D) -> None:
        self._track_session(message.session_id)
        source_ms = int(message.source_time_boot_ms)
        if self.last_source_time_ms is not None and source_ms <= self.last_source_time_ms:
            return
        self.last_source_time_ms = source_ms
        east, north, up = float(message.x_m), float(message.y_m), float(message.z_m)
        self.minimum_altitude_m = up if self.minimum_altitude_m is None else min(self.minimum_altitude_m, up)
        self.maximum_altitude_m = up if self.maximum_altitude_m is None else max(self.maximum_altitude_m, up)
        if self.land_mode_observed and message.has_landed_state and message.landed:
            self.landed_after_flight = True
        for target in self.targets:
            if target["settled"] or target["session_id"] != message.session_id:
                continue
            receipt_ns = int(message.receipt_monotonic_ns)
            if (
                receipt_ns <= target["transmit_monotonic_ns"]
                or source_ms <= target["transmit_source_time_boot_ms"]
            ):
                return
            if target["first_post_command_monotonic_ns"] is None:
                target["first_post_command_monotonic_ns"] = receipt_ns
                target["first_post_command_source_time_boot_ms"] = source_ms
            command = target["commanded_enu"]
            horizontal = ((command["east_m"] - east) ** 2 + (command["north_m"] - north) ** 2) ** 0.5
            vertical = abs(command["up_m"] - up)
            if horizontal <= self.horizontal_tolerance_m and vertical <= self.vertical_tolerance_m:
                if target["arrival_started_ns"] is None:
                    target["arrival_started_ns"] = receipt_ns
                if (receipt_ns - target["arrival_started_ns"]) / 1e9 >= self.settling_time_s:
                    target.update({
                        "settled": True,
                        "settled_monotonic_ns": receipt_ns,
                        "settling_duration_s": (receipt_ns - target["arrival_started_ns"]) / 1e9,
                        "measured_enu": {"east_m": east, "north_m": north, "up_m": up},
                        "horizontal_error_m": horizontal,
                        "vertical_error_m": vertical,
                    })
            else:
                target["arrival_started_ns"] = None
            return

    def _status(self, message: MavlinkTelemetryStatus) -> None:
        self._track_session(message.session_id)
        self.telemetry_fresh = (
            message.health == message.CONNECTED
            and 0.0 <= float(message.telemetry_age_s) <= float(message.freshness_threshold_s)
        )

    def _evaluate(self) -> None:
        if self.finished:
            return
        self.mission_node_observed = self.mission_node_observed or (
            "echorescue_mavlink_waypoint_mission" in set(self.get_node_names())
        )
        expected = ["waypoint-a", "waypoint-b", "return-launch"]
        complete = (
            self.mission_node_observed and not self.session_changed
            and self.guided_observed and self.armed_observed
            and self.target_ids == expected and self.settled_event_ids == expected
            and all(target["settled"] for target in self.targets)
            and self.land_mode_observed and self.landed_after_flight
            and self.disarmed_after_flight and self.telemetry_fresh
        )
        if complete:
            self._finish(True, "independent ROS state and mission events agree on the ordered waypoint mission")
        elif monotonic() - self.started >= self.timeout_s:
            self._finish(False, "timed out waiting for the complete ordered waypoint mission")

    def _finish(self, passed: bool, detail: str) -> None:
        report = {
            "schema_version": "echorescue-waypoint-mission-observer/1.0",
            "milestone": "v0.14.4",
            "status": "PASS" if passed else "FAIL",
            "detail": detail,
            "session_id": self.session_id,
            "session_changed": self.session_changed,
            "mission_node_observed": self.mission_node_observed,
            "guided_observed": self.guided_observed,
            "armed_observed": self.armed_observed,
            "target_ids": self.target_ids,
            "settled_event_ids": self.settled_event_ids,
            "targets": self.targets,
            "land_mode_observed": self.land_mode_observed,
            "landed_after_flight": self.landed_after_flight,
            "disarmed_after_flight": self.disarmed_after_flight,
            "telemetry_fresh": self.telemetry_fresh,
            "minimum_altitude_enu_m": self.minimum_altitude_m,
            "maximum_altitude_enu_m": self.maximum_altitude_m,
            "event_sequence": self.event_sequence,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(serialize_report(report), encoding="utf-8")
        self.passed = passed
        self.finished = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Observe an EchoRescue v0.14.4 waypoint mission")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--horizontal-tolerance", type=float, required=True)
    parser.add_argument("--vertical-tolerance", type=float, required=True)
    parser.add_argument("--settling-time", type=float, required=True)
    return parser


def main(args: list[str] | None = None) -> None:
    parsed, ros_args = build_parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = WaypointMissionObserver(
        parsed.output, parsed.timeout, parsed.horizontal_tolerance,
        parsed.vertical_tolerance, parsed.settling_time,
    )
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

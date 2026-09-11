"""Independent typed-ROS observer for the real v0.14.3 smoke mission."""

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
)
from echorescue_ros.qos import STATUS_QOS, TELEMETRY_QOS


class FlightMissionObserver(Node):
    def __init__(
        self,
        output_path: Path,
        timeout_s: float,
        target_altitude_enu_m: float,
        altitude_tolerance_m: float,
        hover_duration_s: float,
    ) -> None:
        super().__init__("echorescue_flight_mission_observer")
        self.output_path = output_path
        self.timeout_s = timeout_s
        self.target_altitude_enu_m = target_altitude_enu_m
        self.altitude_tolerance_m = altitude_tolerance_m
        self.hover_duration_s = hover_duration_s
        self.started = monotonic()
        self.finished = False
        self.passed = False
        self.session_id = ""
        self.session_changed = False
        self.guided_observed = False
        self.armed_observed = False
        self.land_mode_observed = False
        self.landed_after_flight = False
        self.disarmed_after_flight = False
        self.altitude_reached_at: float | None = None
        self.land_mode_at: float | None = None
        self.minimum_altitude_m: float | None = None
        self.maximum_altitude_m: float | None = None
        self.first_source_time_boot_ms: int | None = None
        self.last_source_time_boot_ms: int | None = None
        self.telemetry_fresh = False
        self.mission_node_observed = False
        self.create_subscription(
            MavlinkVehicleState,
            "/echorescue/mavlink/vehicle_state",
            self._vehicle,
            TELEMETRY_QOS,
        )
        self.create_subscription(
            EchoRescueVehicleState3D,
            "/echorescue/vehicle/state_3d",
            self._state,
            TELEMETRY_QOS,
        )
        self.create_subscription(
            MavlinkTelemetryStatus,
            "/echorescue/mavlink/status",
            self._status,
            STATUS_QOS,
        )
        self.create_timer(0.1, self._evaluate)

    def _track_session(self, session_id: str) -> None:
        if self.session_id and session_id != self.session_id:
            self.session_changed = True
        if session_id:
            self.session_id = session_id

    def _vehicle(self, message: MavlinkVehicleState) -> None:
        self._track_session(message.session_id)
        now = monotonic()
        if message.flight_mode == "GUIDED":
            self.guided_observed = True
        if message.armed:
            self.armed_observed = True
        elif self.armed_observed:
            self.disarmed_after_flight = True
        if message.flight_mode == "LAND" and self.armed_observed:
            self.land_mode_observed = True
            if self.land_mode_at is None:
                self.land_mode_at = now

    def _state(self, message: EchoRescueVehicleState3D) -> None:
        self._track_session(message.session_id)
        source_time = int(message.source_time_boot_ms)
        if self.first_source_time_boot_ms is None:
            self.first_source_time_boot_ms = source_time
        if self.last_source_time_boot_ms is None or source_time > self.last_source_time_boot_ms:
            self.last_source_time_boot_ms = source_time
        altitude = float(message.z_m)
        self.minimum_altitude_m = altitude if self.minimum_altitude_m is None else min(self.minimum_altitude_m, altitude)
        self.maximum_altitude_m = altitude if self.maximum_altitude_m is None else max(self.maximum_altitude_m, altitude)
        if (
            abs(altitude - self.target_altitude_enu_m) <= self.altitude_tolerance_m
            and self.altitude_reached_at is None
        ):
            self.altitude_reached_at = monotonic()
        if self.land_mode_observed and message.has_landed_state and message.landed:
            self.landed_after_flight = True

    def _status(self, message: MavlinkTelemetryStatus) -> None:
        self._track_session(message.session_id)
        self.telemetry_fresh = (
            message.health == message.CONNECTED
            and 0.0 <= float(message.telemetry_age_s) <= float(message.freshness_threshold_s)
        )

    def _graph_ready(self) -> bool:
        return "echorescue_mavlink_flight_mission" in set(self.get_node_names())

    def _evaluate(self) -> None:
        if self.finished:
            return
        self.mission_node_observed = self.mission_node_observed or self._graph_ready()
        hover_observed = (
            self.altitude_reached_at is not None
            and self.land_mode_at is not None
            and self.land_mode_at - self.altitude_reached_at >= self.hover_duration_s
        )
        source_time_advanced = (
            self.first_source_time_boot_ms is not None
            and self.last_source_time_boot_ms is not None
            and self.last_source_time_boot_ms > self.first_source_time_boot_ms
        )
        complete = (
            self.mission_node_observed
            and not self.session_changed
            and self.guided_observed
            and self.armed_observed
            and self.altitude_reached_at is not None
            and hover_observed
            and self.land_mode_observed
            and self.landed_after_flight
            and self.disarmed_after_flight
            and source_time_advanced
            and self.telemetry_fresh
        )
        if complete:
            self._finish(True, "independent ROS telemetry observed the complete flight sequence")
        elif monotonic() - self.started >= self.timeout_s:
            self._finish(False, "timed out waiting for the complete flight sequence")

    def _finish(self, passed: bool, detail: str) -> None:
        hover_observed_s = (
            self.land_mode_at - self.altitude_reached_at
            if self.land_mode_at is not None and self.altitude_reached_at is not None
            else None
        )
        report: dict[str, Any] = {
            "schema_version": "echorescue-flight-mission-observer/1.0",
            "milestone": "v0.14.3",
            "status": "PASS" if passed else "FAIL",
            "detail": detail,
            "session_id": self.session_id,
            "session_changed": self.session_changed,
            "guided_observed": self.guided_observed,
            "armed_observed": self.armed_observed,
            "target_altitude_observed": self.altitude_reached_at is not None,
            "hover_observed_s": hover_observed_s,
            "land_mode_observed": self.land_mode_observed,
            "landed_after_flight": self.landed_after_flight,
            "disarmed_after_flight": self.disarmed_after_flight,
            "minimum_altitude_enu_m": self.minimum_altitude_m,
            "maximum_altitude_enu_m": self.maximum_altitude_m,
            "source_time_boot_ms_first": self.first_source_time_boot_ms,
            "source_time_boot_ms_last": self.last_source_time_boot_ms,
            "telemetry_fresh": self.telemetry_fresh,
            "mission_node_observed": self.mission_node_observed,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(serialize_report(report), encoding="utf-8")
        self.passed = passed
        self.finished = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Observe an EchoRescue v0.14.3 flight mission")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--target-altitude", type=float, required=True)
    parser.add_argument("--altitude-tolerance", type=float, required=True)
    parser.add_argument("--hover-duration", type=float, required=True)
    return parser


def main(args: list[str] | None = None) -> None:
    parsed, ros_args = build_parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = FlightMissionObserver(
        parsed.output,
        parsed.timeout,
        parsed.target_altitude,
        parsed.altitude_tolerance,
        parsed.hover_duration,
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

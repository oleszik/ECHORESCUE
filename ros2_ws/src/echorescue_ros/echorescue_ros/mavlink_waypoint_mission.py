"""Simulation-only v0.14.4 continuous GUIDED waypoint mission node."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import monotonic, monotonic_ns
from typing import Any

import rclpy

from echorescue.continuous_vehicle_state import ned_to_enu
from echorescue.flight_mission import (
    CommandKind,
    CommandRequest,
    MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
    validate_simulation_endpoint,
)
from echorescue.mavlink_telemetry import (
    HeartbeatTelemetry,
    LandedStateTelemetry,
    LocalPositionNedTelemetry,
    TelemetryStatus,
)
from echorescue.waypoint_mission import (
    RelativeTarget,
    TargetRequest,
    WaypointMissionConfig,
    WaypointMissionController,
    serialize_waypoint_report,
)
from echorescue_interfaces.msg import WaypointMissionEvent, WaypointTarget
from echorescue_ros.conversions import waypoint_event_to_msg, waypoint_target_to_msg
from echorescue_ros.mavlink_telemetry_bridge import MavlinkTelemetryBridge
from echorescue_ros.qos import MISSION_QOS


def targets_from_json(value: str) -> tuple[RelativeTarget, ...]:
    """Parse the optional predefined route without adding a new control primitive."""
    decoded = json.loads(value)
    if not isinstance(decoded, list) or not decoded:
        raise ValueError("targets_json must be a non-empty JSON array")
    targets: list[RelativeTarget] = []
    for item in decoded:
        if not isinstance(item, dict):
            raise ValueError("each targets_json entry must be an object")
        targets.append(RelativeTarget(
            str(item["target_id"]), float(item["east_offset_m"]),
            float(item["north_offset_m"]), float(item["altitude_above_launch_m"]),
        ))
    return tuple(targets)


class MavlinkWaypointMission(MavlinkTelemetryBridge):
    """Narrow local-position command path consuming only vehicle telemetry."""

    def __init__(self, output_path: Path) -> None:
        super().__init__(
            node_name="echorescue_mavlink_waypoint_mission",
            role_description="simulation-only continuous GUIDED waypoint mission",
        )
        validate_simulation_endpoint(self.endpoint)
        defaults: dict[str, Any] = {
            "takeoff_altitude_m": 2.0,
            "waypoint_a_east_m": 3.0,
            "waypoint_a_north_m": 0.0,
            "waypoint_a_up_m": 2.0,
            "waypoint_b_east_m": 3.0,
            "waypoint_b_north_m": 3.0,
            "waypoint_b_up_m": 2.5,
            "return_east_m": 0.0,
            "return_north_m": 0.0,
            "return_up_m": 2.0,
            "horizontal_tolerance_m": 0.45,
            "vertical_tolerance_m": 0.35,
            "settling_time_s": 2.0,
            "target_timeout_s": 35.0,
            "progress_timeout_s": 10.0,
            "progress_epsilon_m": 0.15,
            "mission_timeout_s": 180.0,
            "geofence_horizontal_radius_m": 8.0,
            "geofence_min_altitude_m": -0.25,
            "geofence_max_altitude_m": 4.0,
            "preflight_hold_s": 30.0,
            "ready_timeout_s": 45.0,
            "transition_timeout_s": 15.0,
            "takeoff_timeout_s": 35.0,
            "landing_timeout_s": 40.0,
            "cleanup_timeout_s": 40.0,
            "targets_json": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: float(self.get_parameter(name).value)
        targets_json = str(self.get_parameter("targets_json").value)
        targets = targets_from_json(targets_json) if targets_json else (
            RelativeTarget("waypoint-a", value("waypoint_a_east_m"), value("waypoint_a_north_m"), value("waypoint_a_up_m")),
            RelativeTarget("waypoint-b", value("waypoint_b_east_m"), value("waypoint_b_north_m"), value("waypoint_b_up_m")),
            RelativeTarget("return-launch", value("return_east_m"), value("return_north_m"), value("return_up_m")),
        )
        config = WaypointMissionConfig(
            targets=targets,
            takeoff_altitude_m=value("takeoff_altitude_m"),
            horizontal_tolerance_m=value("horizontal_tolerance_m"),
            vertical_tolerance_m=value("vertical_tolerance_m"),
            settling_time_s=value("settling_time_s"),
            target_timeout_s=value("target_timeout_s"),
            progress_timeout_s=value("progress_timeout_s"),
            progress_epsilon_m=value("progress_epsilon_m"),
            mission_timeout_s=value("mission_timeout_s"),
            geofence_horizontal_radius_m=value("geofence_horizontal_radius_m"),
            geofence_min_altitude_m=value("geofence_min_altitude_m"),
            geofence_max_altitude_m=value("geofence_max_altitude_m"),
            preflight_hold_s=value("preflight_hold_s"),
            ready_timeout_s=value("ready_timeout_s"),
            transition_timeout_s=value("transition_timeout_s"),
            takeoff_timeout_s=value("takeoff_timeout_s"),
            landing_timeout_s=value("landing_timeout_s"),
            cleanup_timeout_s=value("cleanup_timeout_s"),
        )
        self.output_path = output_path
        self.controller = WaypointMissionController(config, monotonic_ns())
        self.finished = False
        self.passed = False
        self._report_written = False
        self._published_event_count = 0
        self._extended_stream_session = ""
        self._autopilot_status_text: list[dict[str, Any]] = []
        self._target_pub = self.create_publisher(
            WaypointTarget, "/echorescue/mission/active_target", MISSION_QOS
        )
        self._event_pub = self.create_publisher(
            WaypointMissionEvent, "/echorescue/mission/event", MISSION_QOS
        )
        self.create_timer(0.05, self._advance_mission)

    def _on_heartbeat(self, sample: HeartbeatTelemetry) -> None:
        self.controller.observe_heartbeat(
            session_id=sample.session_id, armed=sample.armed,
            flight_mode=sample.flight_mode, now_ns=sample.receipt_monotonic_ns,
        )
        if self._connection is not None and sample.session_id != self._extended_stream_session:
            self._connection.mav.request_data_stream_send(
                sample.system_id, sample.component_id,
                self._mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1,
            )
            self._extended_stream_session = sample.session_id

    def _on_local_position(self, sample: LocalPositionNedTelemetry) -> None:
        east, north, up = ned_to_enu((sample.north_m, sample.east_m, sample.down_m))
        self.controller.observe_position(
            session_id=sample.session_id, east_m=east, north_m=north, up_m=up,
            source_time_boot_ms=sample.source_time_boot_ms,
            now_ns=sample.receipt_monotonic_ns,
        )

    def _on_landed_state(self, sample: LandedStateTelemetry) -> None:
        self.controller.observe_landed(
            session_id=sample.session_id, landed=sample.landed,
            now_ns=sample.receipt_monotonic_ns,
        )

    def _on_status(self, status: TelemetryStatus) -> None:
        if hasattr(self, "controller"):
            self.controller.observe_status(
                session_id=status.session_id, health=status.health,
                telemetry_age_s=status.telemetry_age_s, now_ns=monotonic_ns(),
            )

    def _on_command_ack(self, fields: dict[str, Any], receipt_ns: int, session_id: str) -> None:
        self.controller.acknowledge(
            session_id=session_id, command_id=int(fields["command"]),
            result=int(fields["result"]), now_ns=receipt_ns,
        )

    def _on_mavlink_message(
        self, message_type: str, fields: dict[str, Any], receipt_ns: int,
        system_id: int, component_id: int,
    ) -> None:
        if message_type == "STATUSTEXT":
            self._autopilot_status_text.append({
                "receipt_monotonic_ns": receipt_ns,
                "severity": int(fields.get("severity", 0)),
                "text": str(fields.get("text", "")).rstrip("\x00"),
            })
            self._autopilot_status_text = self._autopilot_status_text[-32:]

    def _advance_mission(self) -> None:
        if self.finished:
            return
        request = self.controller.tick(monotonic_ns())
        if isinstance(request, CommandRequest):
            self._send_command(request)
        elif isinstance(request, TargetRequest):
            self._send_target(request)
        self._publish_events()
        if self.controller.terminal:
            self._write_report()

    def _send_command(self, request: CommandRequest) -> None:
        connection = self._connection
        if connection is None:
            self.controller.cancel(monotonic_ns(), "command transport unavailable")
            return
        parameters = {
            CommandKind.EXTENDED_STATE_STREAM: (float(MAVLINK_MSG_ID_EXTENDED_SYS_STATE), 500_000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            CommandKind.GUIDED: (1.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            CommandKind.ARM: (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            CommandKind.TAKEOFF: (0.0, 0.0, 0.0, float("nan"), 0.0, 0.0, float(request.target_altitude_enu_m or 0.0)),
            CommandKind.LAND: (0.0, 0.0, 0.0, float("nan"), 0.0, 0.0, 0.0),
        }[request.kind]
        try:
            connection.mav.command_long_send(
                self.core.system_id, self.core.component_id,
                request.command_id, 0, *parameters,
            )
        except (OSError, ConnectionError, EOFError) as error:
            self._close_connection(monotonic_ns(), f"MAVLink command transport lost: {error}")

    def _send_target(self, request: TargetRequest) -> None:
        connection = self._connection
        now_ns = monotonic_ns()
        if connection is None:
            self.controller.target_transmitted(
                session_id=request.session_id, success=False, now_ns=now_ns,
                detail="MAVLink transport unavailable",
            )
            return
        setpoint = request.ned
        try:
            connection.mav.set_position_target_local_ned_send(
                0, self.core.system_id, self.core.component_id,
                setpoint.coordinate_frame, setpoint.type_mask,
                setpoint.north_m, setpoint.east_m, setpoint.down_m,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                setpoint.yaw_rad, 0.0,
            )
        except (OSError, ConnectionError, EOFError, ValueError) as error:
            self.controller.target_transmitted(
                session_id=request.session_id, success=False, now_ns=now_ns,
                detail=str(error),
            )
            self._close_connection(now_ns, f"MAVLink target transport lost: {error}")
            return
        accepted = self.controller.target_transmitted(
            session_id=request.session_id, success=True, now_ns=now_ns,
            detail="pymavlink send returned without error; no COMMAND_ACK exists for this message",
        )
        source_ms = self.controller.latest_source_time_boot_ms
        if accepted and source_ms is not None:
            self._target_pub.publish(waypoint_target_to_msg(
                request, self.get_clock().now().to_msg(), sequence=len(self.controller.targets),
                transmit_monotonic_ns=now_ns, transmit_source_time_boot_ms=source_ms,
            ))

    def _publish_events(self) -> None:
        for event in self.controller.events[self._published_event_count:]:
            self._event_pub.publish(waypoint_event_to_msg(event, self.get_clock().now().to_msg()))
        self._published_event_count = len(self.controller.events)

    def cancel_for_shutdown(self) -> None:
        self.controller.cancel(monotonic_ns(), "mission process interrupted")

    def _write_report(self) -> None:
        if self._report_written:
            return
        report = self.controller.report()
        report["autopilot_status_text"] = list(self._autopilot_status_text)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(serialize_waypoint_report(report), encoding="utf-8")
        self.passed = self.controller.succeeded
        self.finished = True
        self._report_written = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the EchoRescue v0.14.4 waypoint mission")
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(args: list[str] | None = None) -> None:
    parsed, ros_args = build_parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = MavlinkWaypointMission(parsed.output)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.cancel_for_shutdown()
        deadline = monotonic() + node.controller.config.cleanup_timeout_s + 1.0
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

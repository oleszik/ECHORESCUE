"""Simulation-only v0.14.3 MAVLink arm/takeoff/hover/land runner."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic, monotonic_ns
from typing import Any

import rclpy

from echorescue.flight_mission import (
    CommandKind,
    CommandRequest,
    FlightMissionController,
    MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
    MissionConfig,
    serialize_mission_report,
    validate_simulation_endpoint,
)
from echorescue.mavlink_telemetry import (
    HeartbeatTelemetry,
    LandedStateTelemetry,
    LocalPositionNedTelemetry,
    TelemetryStatus,
)
from echorescue_ros.mavlink_telemetry_bridge import MavlinkTelemetryBridge


class MavlinkFlightMission(MavlinkTelemetryBridge):
    """A separate command-capable executable built on the telemetry bridge."""

    def __init__(self, output_path: Path) -> None:
        super().__init__(
            node_name="echorescue_mavlink_flight_mission",
            role_description="simulation-only closed-loop MAVLink mission",
        )
        validate_simulation_endpoint(self.endpoint)
        self.declare_parameter("target_altitude_enu_m", 2.0)
        self.declare_parameter("altitude_tolerance_m", 0.35)
        self.declare_parameter("hover_duration_s", 3.0)
        self.declare_parameter("preflight_hold_s", 30.0)
        self.declare_parameter("ready_timeout_s", 45.0)
        self.declare_parameter("transition_timeout_s", 15.0)
        self.declare_parameter("takeoff_timeout_s", 30.0)
        self.declare_parameter("landing_timeout_s", 30.0)
        self.declare_parameter("cleanup_timeout_s", 30.0)
        config = MissionConfig(
            target_altitude_enu_m=float(self.get_parameter("target_altitude_enu_m").value),
            altitude_tolerance_m=float(self.get_parameter("altitude_tolerance_m").value),
            hover_duration_s=float(self.get_parameter("hover_duration_s").value),
            preflight_hold_s=float(self.get_parameter("preflight_hold_s").value),
            ready_timeout_s=float(self.get_parameter("ready_timeout_s").value),
            transition_timeout_s=float(self.get_parameter("transition_timeout_s").value),
            takeoff_timeout_s=float(self.get_parameter("takeoff_timeout_s").value),
            landing_timeout_s=float(self.get_parameter("landing_timeout_s").value),
            cleanup_timeout_s=float(self.get_parameter("cleanup_timeout_s").value),
        )
        self.output_path = output_path
        self.controller = FlightMissionController(config, monotonic_ns())
        self.finished = False
        self.passed = False
        self._report_written = False
        self._extended_stream_session = ""
        self._autopilot_status_text: list[dict[str, Any]] = []
        self.create_timer(0.05, self._advance_mission)
        self.get_logger().info(
            f"mission target is z={config.target_altitude_enu_m:.2f} m in echorescue/world_enu"
        )

    def _on_heartbeat(self, sample: HeartbeatTelemetry) -> None:
        self.controller.observe_heartbeat(
            session_id=sample.session_id,
            armed=sample.armed,
            flight_mode=sample.flight_mode,
            now_ns=sample.receipt_monotonic_ns,
        )
        if self._connection is not None and sample.session_id != self._extended_stream_session:
            self._connection.mav.request_data_stream_send(
                sample.system_id,
                sample.component_id,
                self._mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                2,
                1,
            )
            self._extended_stream_session = sample.session_id
            self.get_logger().info("requested MAV_DATA_STREAM_EXTENDED_STATUS at 2 Hz")

    def _on_mavlink_message(
        self,
        message_type: str,
        fields: dict[str, Any],
        receipt_ns: int,
        system_id: int,
        component_id: int,
    ) -> None:
        if message_type != "STATUSTEXT":
            return
        detail = str(fields.get("text", "")).rstrip("\x00")
        self._autopilot_status_text.append({
            "receipt_monotonic_ns": receipt_ns,
            "severity": int(fields.get("severity", 0)),
            "text": detail,
        })
        self._autopilot_status_text = self._autopilot_status_text[-32:]
        self.get_logger().warning(f"ArduPilot STATUSTEXT: {detail}")

    def _on_local_position(self, sample: LocalPositionNedTelemetry) -> None:
        self.controller.observe_position(
            session_id=sample.session_id,
            altitude_enu_m=-sample.down_m,
            now_ns=sample.receipt_monotonic_ns,
        )

    def _on_landed_state(self, sample: LandedStateTelemetry) -> None:
        self.controller.observe_landed(
            session_id=sample.session_id,
            landed=sample.landed,
            now_ns=sample.receipt_monotonic_ns,
        )

    def _on_status(self, status: TelemetryStatus) -> None:
        if not hasattr(self, "controller"):
            return
        self.controller.observe_status(
            session_id=status.session_id,
            health=status.health,
            telemetry_age_s=status.telemetry_age_s,
            now_ns=monotonic_ns(),
        )

    def _on_command_ack(
        self,
        fields: dict[str, Any],
        receipt_ns: int,
        session_id: str,
    ) -> None:
        self.controller.acknowledge(
            session_id=session_id,
            command_id=int(fields["command"]),
            result=int(fields["result"]),
            now_ns=receipt_ns,
        )

    def _advance_mission(self) -> None:
        if self.finished:
            return
        request = self.controller.tick(monotonic_ns())
        if request is not None:
            self._send_command(request)
        if self.controller.terminal:
            self._write_report()

    def _send_command(self, request: CommandRequest) -> None:
        connection = self._connection
        if connection is None:
            self.controller.cancel(monotonic_ns(), "command transport unavailable")
            return
        parameters = {
            CommandKind.EXTENDED_STATE_STREAM: (
                float(MAVLINK_MSG_ID_EXTENDED_SYS_STATE),
                500_000.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            CommandKind.GUIDED: (1.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            CommandKind.ARM: (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            CommandKind.TAKEOFF: (
                0.0,
                0.0,
                0.0,
                float("nan"),
                0.0,
                0.0,
                float(request.target_altitude_enu_m or 0.0),
            ),
            CommandKind.LAND: (0.0, 0.0, 0.0, float("nan"), 0.0, 0.0, 0.0),
        }[request.kind]
        try:
            connection.mav.command_long_send(
                self.core.system_id,
                self.core.component_id,
                request.command_id,
                0,
                *parameters,
            )
        except (OSError, ConnectionError, EOFError) as error:
            self._close_connection(monotonic_ns(), f"MAVLink command transport lost: {error}")
            return
        self.get_logger().info(
            f"sent {request.kind.value} command={request.command_id}; awaiting COMMAND_ACK and telemetry"
        )

    def cancel_for_shutdown(self) -> None:
        self.controller.cancel(monotonic_ns(), "mission process interrupted")

    def _write_report(self) -> None:
        if self._report_written:
            return
        report = self.controller.report()
        report["autopilot_status_text"] = list(self._autopilot_status_text)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(serialize_mission_report(report), encoding="utf-8")
        self.passed = self.controller.succeeded
        self.finished = True
        self._report_written = True
        level = self.get_logger().info if self.passed else self.get_logger().error
        level(f"mission finished with {report['status']}: {report['phase']}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the EchoRescue v0.14.3 simulation flight milestone")
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(args: list[str] | None = None) -> None:
    parsed, ros_args = build_parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = MavlinkFlightMission(parsed.output)
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

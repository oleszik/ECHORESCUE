"""Independent typed ROS subscriber used by the real v0.14.1 smoke test."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from echorescue.sim_integration import serialize_report
from echorescue_interfaces.msg import (
    EchoRescueVehicleState3D,
    MavlinkGlobalPosition,
    MavlinkLocalPositionNed,
    MavlinkTelemetryStatus,
    MavlinkVehicleState,
)


TELEMETRY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=32,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
REQUIRED_TOPICS = {
    "/echorescue/mavlink/vehicle_state": "echorescue_interfaces/msg/MavlinkVehicleState",
    "/echorescue/mavlink/local_position_ned": "echorescue_interfaces/msg/MavlinkLocalPositionNed",
    "/echorescue/mavlink/global_position": "echorescue_interfaces/msg/MavlinkGlobalPosition",
    "/echorescue/mavlink/status": "echorescue_interfaces/msg/MavlinkTelemetryStatus",
}
CONVERTED_TOPIC = "/echorescue/vehicle/state_3d"
CONVERTED_TYPE = "echorescue_interfaces/msg/EchoRescueVehicleState3D"


class TelemetryObserver(Node):
    def __init__(self, output_path: Path, timeout_s: float, minimum_samples: int, require_converted_state: bool = False) -> None:
        super().__init__("echorescue_mavlink_telemetry_observer")
        self.output_path = output_path
        self.timeout_s = timeout_s
        self.minimum_samples = minimum_samples
        self.require_converted_state = require_converted_state
        self.started = monotonic()
        self.finished = False
        self.passed = False
        self.vehicle_state: MavlinkVehicleState | None = None
        self.local_samples: list[MavlinkLocalPositionNed] = []
        self.global_sample: MavlinkGlobalPosition | None = None
        self.status: MavlinkTelemetryStatus | None = None
        self.converted_samples: list[EchoRescueVehicleState3D] = []
        self.create_subscription(
            MavlinkVehicleState,
            "/echorescue/mavlink/vehicle_state",
            self._vehicle,
            TELEMETRY_QOS,
        )
        self.create_subscription(
            EchoRescueVehicleState3D,
            CONVERTED_TOPIC,
            self._converted,
            TELEMETRY_QOS,
        )
        self.create_subscription(
            MavlinkLocalPositionNed,
            "/echorescue/mavlink/local_position_ned",
            self._local,
            TELEMETRY_QOS,
        )
        self.create_subscription(
            MavlinkGlobalPosition,
            "/echorescue/mavlink/global_position",
            self._global,
            TELEMETRY_QOS,
        )
        self.create_subscription(
            MavlinkTelemetryStatus,
            "/echorescue/mavlink/status",
            self._health,
            STATUS_QOS,
        )
        self.create_timer(0.1, self._evaluate)

    def _vehicle(self, message: MavlinkVehicleState) -> None:
        self.vehicle_state = message

    def _local(self, message: MavlinkLocalPositionNed) -> None:
        if not self.local_samples or message.source_time_boot_ms > self.local_samples[-1].source_time_boot_ms:
            self.local_samples.append(message)

    def _global(self, message: MavlinkGlobalPosition) -> None:
        self.global_sample = message

    def _health(self, message: MavlinkTelemetryStatus) -> None:
        self.status = message

    def _converted(self, message: EchoRescueVehicleState3D) -> None:
        if not self.converted_samples:
            self.converted_samples.append(message)
            return
        previous = self.converted_samples[-1]
        if message.session_id == previous.session_id and message.source_time_boot_ms > previous.source_time_boot_ms:
            self.converted_samples.append(message)
        elif message.session_id != previous.session_id:
            self.converted_samples = [message]

    def _graph_ready(self) -> tuple[bool, list[str], list[str]]:
        nodes = set(self.get_node_names())
        topics = dict(self.get_topic_names_and_types())
        required_topics = dict(REQUIRED_TOPICS)
        if self.require_converted_state:
            required_topics[CONVERTED_TOPIC] = CONVERTED_TYPE
        missing_topics = sorted(
            name for name, message_type in required_topics.items()
            if message_type not in topics.get(name, [])
        )
        missing_nodes = [] if "echorescue_mavlink_telemetry_bridge" in nodes else [
            "echorescue_mavlink_telemetry_bridge"
        ]
        return not missing_nodes and not missing_topics, missing_nodes, missing_topics

    def _matching_conversions(self) -> int:
        sources = {
            (sample.session_id, int(sample.source_time_boot_ms)): sample
            for sample in self.local_samples
        }
        matches = 0
        for converted in self.converted_samples:
            source = sources.get((converted.session_id, int(converted.source_time_boot_ms)))
            if source is None:
                continue
            expected = (
                float(source.east_m), float(source.north_m), -float(source.down_m),
                float(source.velocity_east_m_s), float(source.velocity_north_m_s),
                -float(source.velocity_down_m_s),
            )
            actual = (
                float(converted.x_m), float(converted.y_m), float(converted.z_m),
                float(converted.vx_m_s), float(converted.vy_m_s), float(converted.vz_m_s),
            )
            if all(abs(left - right) <= 1e-9 for left, right in zip(actual, expected)):
                matches += 1
        return matches

    def _evaluate(self) -> None:
        if self.finished:
            return
        graph_ready, missing_nodes, missing_topics = self._graph_ready()
        timestamps = [int(sample.source_time_boot_ms) for sample in self.local_samples]
        advancing = len(timestamps) >= self.minimum_samples and timestamps[-1] > timestamps[0]
        converted_timestamps = [int(sample.source_time_boot_ms) for sample in self.converted_samples]
        converted_ready = (
            not self.require_converted_state
            or (
                len(converted_timestamps) >= self.minimum_samples
                and converted_timestamps[-1] > converted_timestamps[0]
                and all(sample.position_valid and sample.velocity_valid for sample in self.converted_samples)
            )
        )
        matching_conversions = self._matching_conversions()
        conversion_correct = not self.require_converted_state or matching_conversions >= self.minimum_samples
        age_fresh = (
            self.status is not None
            and self.status.health == self.status.CONNECTED
            and 0.0 <= self.status.telemetry_age_s <= self.status.freshness_threshold_s
        )
        if graph_ready and self.vehicle_state is not None and advancing and self.global_sample is not None and age_fresh and converted_ready and conversion_correct:
            self._finish(True, "typed advancing MAVLink telemetry observed", missing_nodes, missing_topics)
        elif monotonic() - self.started >= self.timeout_s:
            self._finish(False, "timed out waiting for complete typed telemetry", missing_nodes, missing_topics)

    def _finish(self, passed: bool, detail: str, missing_nodes: list[str], missing_topics: list[str]) -> None:
        timestamps = [int(sample.source_time_boot_ms) for sample in self.local_samples]
        converted_timestamps = [int(sample.source_time_boot_ms) for sample in self.converted_samples]
        converted = self.converted_samples[-1] if self.converted_samples else None
        matching_conversions = self._matching_conversions()
        report: dict[str, Any] = {
            "schema_version": "echorescue-mavlink-telemetry-observer/1.1",
            "status": "PASS" if passed else "FAIL",
            "detail": detail,
            "missing_nodes": missing_nodes,
            "missing_topics": missing_topics,
            "vehicle_state_received": self.vehicle_state is not None,
            "global_position_received": self.global_sample is not None,
            "local_position_samples": len(timestamps),
            "local_source_time_boot_ms_first": timestamps[0] if timestamps else None,
            "local_source_time_boot_ms_last": timestamps[-1] if timestamps else None,
            "converted_state_samples": len(converted_timestamps),
            "converted_session_id": converted.session_id if converted is not None else None,
            "converted_source_time_boot_ms_first": converted_timestamps[0] if converted_timestamps else None,
            "converted_source_time_boot_ms_last": converted_timestamps[-1] if converted_timestamps else None,
            "converted_receipt_monotonic_ns": int(converted.receipt_monotonic_ns) if converted is not None else None,
            "coordinate_conversion_matches": matching_conversions,
            "coordinate_conversion_consistent": matching_conversions >= self.minimum_samples,
            "telemetry_age_s": float(self.status.telemetry_age_s) if self.status is not None else None,
            "freshness_threshold_s": float(self.status.freshness_threshold_s) if self.status is not None else None,
            "health": int(self.status.health) if self.status is not None else None,
            "example": {
                "armed": bool(self.vehicle_state.armed) if self.vehicle_state is not None else None,
                "flight_mode": self.vehicle_state.flight_mode if self.vehicle_state is not None else None,
                "system_id": int(self.vehicle_state.system_id) if self.vehicle_state is not None else None,
                "component_id": int(self.vehicle_state.component_id) if self.vehicle_state is not None else None,
                "frame_id": self.local_samples[-1].header.frame_id if self.local_samples else None,
                "north_m": float(self.local_samples[-1].north_m) if self.local_samples else None,
                "east_m": float(self.local_samples[-1].east_m) if self.local_samples else None,
                "down_m": float(self.local_samples[-1].down_m) if self.local_samples else None,
                "output_frame": converted.output_frame if converted is not None else None,
                "source_body_frame": converted.source_body_frame if converted is not None else None,
                "output_body_frame": converted.output_body_frame if converted is not None else None,
                "x_m": float(converted.x_m) if converted is not None else None,
                "y_m": float(converted.y_m) if converted is not None else None,
                "z_m": float(converted.z_m) if converted is not None else None,
                "has_attitude": bool(converted.has_attitude) if converted is not None else None,
                "yaw_rad": float(converted.yaw_rad) if converted is not None and converted.has_attitude else None,
                "has_heading": bool(converted.has_heading) if converted is not None else None,
                "heading_deg": float(converted.heading_deg) if converted is not None and converted.has_heading else None,
                "has_landed_state": bool(converted.has_landed_state) if converted is not None else None,
                "landed": bool(converted.landed) if converted is not None and converted.has_landed_state else None,
                "stationary": (
                    abs(float(converted.vx_m_s)) < 0.05
                    and abs(float(converted.vy_m_s)) < 0.05
                    and abs(float(converted.vz_m_s)) < 0.05
                ) if converted is not None else None,
            },
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(serialize_report(report), encoding="utf-8")
        self.passed = passed
        self.finished = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Observe typed EchoRescue MAVLink telemetry")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--minimum-samples", type=int, default=2)
    parser.add_argument("--require-converted-state", action="store_true")
    return parser


def main(args: list[str] | None = None) -> None:
    parsed, ros_args = build_parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = TelemetryObserver(parsed.output, parsed.timeout, parsed.minimum_samples, parsed.require_converted_state)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if not node.finished:
            node._finish(False, "observer interrupted", [], [])
    finally:
        passed = node.passed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()

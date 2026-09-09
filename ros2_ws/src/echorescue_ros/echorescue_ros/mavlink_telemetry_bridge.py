"""Receive-only MAVLink-to-ROS 2 telemetry bridge through v0.14.2."""

from time import monotonic_ns
from typing import Any
from uuid import uuid4

from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from echorescue.continuous_vehicle_state import ContinuousStateAssembler
from echorescue.mavlink_telemetry import TelemetryCore
from echorescue_interfaces.msg import (
    EchoRescueVehicleState3D,
    MavlinkGlobalPosition,
    MavlinkLocalPositionNed,
    MavlinkTelemetryStatus,
    MavlinkVehicleState,
)
from echorescue_ros.conversions import (
    continuous_vehicle_state_to_msg,
    mavlink_global_position_to_msg,
    mavlink_local_position_to_msg,
    mavlink_status_to_msg,
    mavlink_vehicle_state_to_msg,
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


class MavlinkTelemetryBridge(Node):
    def __init__(self) -> None:
        super().__init__("echorescue_mavlink_telemetry_bridge")
        self.declare_parameter("endpoint", "tcp:127.0.0.1:5760")
        self.declare_parameter("stream_rate_hz", 10)
        self.declare_parameter("freshness_threshold_s", 1.0)
        self.declare_parameter("disconnect_threshold_s", 3.0)
        self.declare_parameter("reconnect_interval_s", 1.0)
        self.declare_parameter("vehicle_id", "iris-1")
        self.declare_parameter("source_frame", "mavlink/local_ned")
        self.declare_parameter("output_frame", "echorescue/world_enu")
        self.declare_parameter("source_body_frame", "mavlink/body_frd")
        self.declare_parameter("output_body_frame", "echorescue/body_flu")
        self.declare_parameter("world_origin_policy", "ardupilot_local_ned_at_sitl_startup")
        self.declare_parameter("angle_convention", "right_handed_radians_ccw_from_east")
        self.endpoint = str(self.get_parameter("endpoint").value)
        self.stream_rate_hz = int(self.get_parameter("stream_rate_hz").value)
        self.freshness_threshold_s = float(self.get_parameter("freshness_threshold_s").value)
        self.disconnect_threshold_s = float(self.get_parameter("disconnect_threshold_s").value)
        self.reconnect_interval_s = float(self.get_parameter("reconnect_interval_s").value)
        if self.stream_rate_hz <= 0:
            raise ValueError("stream_rate_hz must be positive")
        if self.disconnect_threshold_s <= self.freshness_threshold_s:
            raise ValueError("disconnect_threshold_s must exceed freshness_threshold_s")
        if self.reconnect_interval_s <= 0:
            raise ValueError("reconnect_interval_s must be positive")

        self.core = TelemetryCore(f"mavlink-{uuid4().hex}", self.freshness_threshold_s)
        self.state_assembler = ContinuousStateAssembler(
            str(self.get_parameter("vehicle_id").value),
            source_frame=str(self.get_parameter("source_frame").value),
            output_frame=str(self.get_parameter("output_frame").value),
            source_body_frame=str(self.get_parameter("source_body_frame").value),
            output_body_frame=str(self.get_parameter("output_body_frame").value),
            world_origin_policy=str(self.get_parameter("world_origin_policy").value),
            angle_convention=str(self.get_parameter("angle_convention").value),
        )
        self._connection: Any | None = None
        self._connection_started_ns = 0
        self._last_connect_attempt_ns = 0
        self._last_heartbeat_ns = 0
        self._stream_requested = False
        self._vehicle_pub = self.create_publisher(
            MavlinkVehicleState, "/echorescue/mavlink/vehicle_state", TELEMETRY_QOS
        )
        self._local_pub = self.create_publisher(
            MavlinkLocalPositionNed, "/echorescue/mavlink/local_position_ned", TELEMETRY_QOS
        )
        self._global_pub = self.create_publisher(
            MavlinkGlobalPosition, "/echorescue/mavlink/global_position", TELEMETRY_QOS
        )
        self._status_pub = self.create_publisher(
            MavlinkTelemetryStatus, "/echorescue/mavlink/status", STATUS_QOS
        )
        self._continuous_state_pub = self.create_publisher(
            EchoRescueVehicleState3D, "/echorescue/vehicle/state_3d", TELEMETRY_QOS
        )
        self.create_timer(0.02, self._poll_mavlink)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(f"receive-only MAVLink bridge configured for {self.endpoint}")

    def _connect(self, now_ns: int) -> None:
        interval_ns = int(self.reconnect_interval_s * 1_000_000_000)
        if now_ns - self._last_connect_attempt_ns < interval_ns:
            return
        self._last_connect_attempt_ns = now_ns
        try:
            self._connection = mavutil.mavlink_connection(self.endpoint)
        except (OSError, ConnectionError) as error:
            self.core.disconnect(now_ns, f"MAVLink connection failed: {error}")
            return
        self._connection_started_ns = now_ns
        self._last_heartbeat_ns = 0
        self._stream_requested = False
        self.get_logger().info("MAVLink transport opened; waiting for heartbeat")

    def _poll_mavlink(self) -> None:
        now_ns = monotonic_ns()
        if self._connection is None:
            self._connect(now_ns)
            return
        try:
            for _ in range(64):
                message = self._connection.recv_match(blocking=False)
                if message is None:
                    break
                self._handle_message(message, monotonic_ns())
        except (OSError, ConnectionError, EOFError) as error:
            self._close_connection(now_ns, f"MAVLink connection lost: {error}")
            return
        heartbeat_reference = self._last_heartbeat_ns or self._connection_started_ns
        if now_ns - heartbeat_reference > int(self.disconnect_threshold_s * 1_000_000_000):
            self._close_connection(now_ns, "MAVLink heartbeat timeout")

    def _handle_message(self, message: Any, receipt_ns: int) -> None:
        message_type = str(message.get_type())
        fields = message.to_dict()
        source_sequence = int(message.get_seq())
        system_id = int(message.get_srcSystem())
        component_id = int(message.get_srcComponent())
        stamp = self.get_clock().now().to_msg()
        if message_type == "HEARTBEAT":
            sample = self.core.ingest_heartbeat(
                fields,
                source_sequence=source_sequence,
                system_id=system_id,
                component_id=component_id,
                receipt_monotonic_ns=receipt_ns,
            )
            if sample is None:
                return
            self._last_heartbeat_ns = receipt_ns
            self.state_assembler.ingest_heartbeat(sample)
            self._vehicle_pub.publish(mavlink_vehicle_state_to_msg(sample, stamp))
            if not self._stream_requested:
                self._connection.mav.request_data_stream_send(
                    system_id,
                    component_id,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    self.stream_rate_hz,
                    1,
                )
                self._stream_requested = True
                self.get_logger().info(f"requested MAV_DATA_STREAM_POSITION at {self.stream_rate_hz} Hz")
        elif message_type == "LOCAL_POSITION_NED":
            local = self.core.ingest_local_position(
                fields,
                source_sequence=source_sequence,
                system_id=system_id,
                component_id=component_id,
                receipt_monotonic_ns=receipt_ns,
            )
            if local is not None:
                self._local_pub.publish(mavlink_local_position_to_msg(local, stamp))
                converted = self.state_assembler.convert(local, self.core.status(receipt_ns))
                if converted is not None:
                    self._continuous_state_pub.publish(continuous_vehicle_state_to_msg(converted, stamp))
        elif message_type == "GLOBAL_POSITION_INT":
            global_position = self.core.ingest_global_position(
                fields,
                source_sequence=source_sequence,
                system_id=system_id,
                component_id=component_id,
                receipt_monotonic_ns=receipt_ns,
            )
            if global_position is not None:
                self.state_assembler.ingest_heading(global_position)
                self._global_pub.publish(mavlink_global_position_to_msg(global_position, stamp))
        elif message_type == "ATTITUDE":
            attitude = self.core.ingest_attitude(
                fields, source_sequence=source_sequence, system_id=system_id,
                component_id=component_id, receipt_monotonic_ns=receipt_ns,
            )
            if attitude is not None:
                self.state_assembler.ingest_attitude(attitude)
        elif message_type == "EXTENDED_SYS_STATE":
            landed = self.core.ingest_extended_system_state(
                fields, source_sequence=source_sequence, system_id=system_id,
                component_id=component_id, receipt_monotonic_ns=receipt_ns,
            )
            if landed is not None:
                self.state_assembler.ingest_landed_state(landed)

    def _publish_status(self) -> None:
        status = self.core.status(monotonic_ns())
        self._status_pub.publish(mavlink_status_to_msg(status, self.get_clock().now().to_msg()))

    def _close_connection(self, now_ns: int, detail: str) -> None:
        if self._connection is not None:
            try:
                self._connection.close()
            except OSError:
                pass
        self._connection = None
        self._connection_started_ns = 0
        self._last_heartbeat_ns = 0
        self._stream_requested = False
        self.core.disconnect(now_ns, detail)
        self.get_logger().warning(detail)

    def destroy_node(self) -> bool:
        if self._connection is not None:
            self._close_connection(monotonic_ns(), "bridge shutdown")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MavlinkTelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Adapt only Gazebo lidar samples to the typed v0.15.1 ROS boundary."""

from __future__ import annotations

import argparse
from math import isfinite
import queue
import re
import subprocess
import threading
from time import monotonic_ns

import rclpy
from rclpy.node import Node

from echorescue_interfaces.msg import EchoRescueVehicleState3D, RangeObstacleObservation
from echorescue_ros.qos import MISSION_QOS, TELEMETRY_QOS


FLOAT = r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|inf|nan)"


def parse_scan(text: str, session_id: str, sequence: int, receipt_ns: int) -> RangeObstacleObservation:
    def scalar(name: str) -> float:
        match = re.search(rf"^{name}: {FLOAT}$", text, re.MULTILINE)
        if not match:
            raise ValueError(f"missing lidar field {name}")
        return float(match.group(1))

    stamp = re.search(r"stamp \{(.*?)\n\s*\}", text, re.DOTALL)
    frame = re.search(r'^frame: "([^"]+)"$', text, re.MULTILINE)
    if not stamp or not frame:
        raise ValueError("lidar sample lacks timestamp or frame")
    seconds = re.search(r"\bsec: (\d+)", stamp.group(1))
    nanoseconds = re.search(r"\bnsec: (\d+)", stamp.group(1))
    minimum, maximum = scalar("range_min"), scalar("range_max")
    raw_ranges = [float(value) for value in re.findall(rf"^ranges: {FLOAT}$", text, re.MULTILINE)]
    count = int(scalar("count"))
    if len(raw_ranges) != count:
        raise ValueError("lidar range count mismatch")
    message = RangeObstacleObservation()
    message.header.frame_id = "iris_range_link"
    message.session_id = session_id
    message.sequence = sequence
    message.sensor_time_ns = (int(seconds.group(1)) if seconds else 0) * 1_000_000_000 + (int(nanoseconds.group(1)) if nanoseconds else 0)
    message.receipt_monotonic_ns = receipt_ns
    message.sensor_frame = "iris_range_link"
    message.angle_min_rad = scalar("angle_min")
    message.angle_increment_rad = scalar("angle_step")
    message.range_min_m = minimum
    message.range_max_m = maximum
    # Gazebo uses +inf for a valid no-return beam. The typed boundary encodes
    # that as the declared maximum; all other non-finite values are rejected.
    message.ranges_m = [maximum if value == float("inf") else value for value in raw_ranges]
    if any(not isfinite(value) for value in message.ranges_m):
        raise ValueError("lidar sample contains an invalid non-finite range")
    return message


class GazeboRangeSensorBridge(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("echorescue_gazebo_range_sensor_bridge")
        self.session_id = ""
        self.sequence = 0
        self.pending: queue.SimpleQueue[str] = queue.SimpleQueue()
        self.publisher = self.create_publisher(RangeObstacleObservation, "/echorescue/sensors/range_observation", MISSION_QOS)
        self.create_subscription(EchoRescueVehicleState3D, "/echorescue/vehicle/state_3d", self._state, TELEMETRY_QOS)
        self.process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", topic], stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, start_new_session=True,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()
        self.create_timer(0.02, self._publish)

    def _state(self, message: EchoRescueVehicleState3D) -> None:
        self.session_id = message.session_id

    def _read(self) -> None:
        assert self.process.stdout is not None
        block: list[str] = []
        for line in self.process.stdout:
            if line.startswith("header {") and block:
                self.pending.put("".join(block))
                block = []
            block.append(line)

    def _publish(self) -> None:
        while not self.pending.empty():
            raw = self.pending.get()
            if not self.session_id:
                continue
            try:
                self.sequence += 1
                message = parse_scan(raw, self.session_id, self.sequence, monotonic_ns())
            except ValueError as error:
                self.get_logger().warning(str(error))
                continue
            message.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)

    def close(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/echorescue/sensors/obstacle_scan")
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = GazeboRangeSensorBridge(parsed.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

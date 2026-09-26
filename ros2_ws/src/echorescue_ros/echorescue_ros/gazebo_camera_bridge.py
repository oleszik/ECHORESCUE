"""Narrow Gazebo-rendered RGB image to sensor_msgs adapter for v0.16.1."""

from __future__ import annotations

import argparse
import queue
from time import monotonic_ns

from gz.msgs10.image_pb2 import Image as GazeboImage
from gz.msgs10.image_pb2 import PixelFormatType
from gz.transport13 import Node as GazeboNode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from echorescue_ros.qos import IMAGE_QOS


class GazeboCameraBridge(Node):
    def __init__(self, topic: str, frame: str, fx: float, fy: float,
                 cx: float, cy: float) -> None:
        super().__init__("echorescue_gazebo_camera_bridge")
        self.frame = frame
        self.sequence = 0
        self.pending: queue.Queue[tuple[GazeboImage, int]] = queue.Queue(maxsize=2)
        self.image_publisher = self.create_publisher(Image, "/echorescue/camera/image_raw", IMAGE_QOS)
        self.info_publisher = self.create_publisher(CameraInfo, "/echorescue/camera/camera_info", IMAGE_QOS)
        self.gz_node = GazeboNode()
        if not self.gz_node.subscribe(GazeboImage, topic, self._receive):
            raise RuntimeError("camera_topic_subscription_failed")
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.create_timer(0.02, self._publish)

    def _receive(self, message: GazeboImage) -> None:
        item = (message, monotonic_ns())
        if self.pending.full():
            try:
                self.pending.get_nowait()
            except queue.Empty:
                pass
        self.pending.put_nowait(item)

    def _publish(self) -> None:
        while not self.pending.empty():
            source, _ = self.pending.get_nowait()
            formats = {
                PixelFormatType.RGB_INT8: "rgb8",
                PixelFormatType.BGR_INT8: "bgr8",
            }
            encoding = formats.get(source.pixel_format_type)
            if encoding is None:
                self.get_logger().warning("unsupported_camera_pixel_format")
                continue
            self.sequence += 1
            image = Image()
            if source.header.HasField("stamp"):
                image.header.stamp.sec = int(source.header.stamp.sec)
                image.header.stamp.nanosec = int(source.header.stamp.nsec)
            else:
                image.header.stamp = self.get_clock().now().to_msg()
            image.header.frame_id = self.frame
            image.height, image.width = source.height, source.width
            image.encoding = encoding
            image.is_bigendian = False
            image.step = source.step
            image.data = bytes(source.data)
            info = CameraInfo()
            info.header = image.header
            info.height, info.width = image.height, image.width
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_publisher.publish(info)
            self.image_publisher.publish(image)


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/echorescue/camera/image")
    parser.add_argument("--frame", default="echorescue_camera_optical")
    parser.add_argument("--fx", type=float, default=277.128)
    parser.add_argument("--fy", type=float, default=277.128)
    parser.add_argument("--cx", type=float, default=160.0)
    parser.add_argument("--cy", type=float, default=120.0)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = GazeboCameraBridge(parsed.topic, parsed.frame, parsed.fx, parsed.fy, parsed.cx, parsed.cy)
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

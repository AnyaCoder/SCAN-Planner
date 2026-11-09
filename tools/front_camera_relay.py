#!/usr/bin/env python3
"""Pull the Go2 front-camera JPEG stream and publish a compressed ROS image."""

import argparse
import os
import threading
import time
from urllib.request import urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class FrontCameraRelay(Node):
    def __init__(self, url, topic, frame_id, fps):
        super().__init__("go2_front_camera_relay")
        self.url = url
        self.fps = fps
        self.frame_id = frame_id
        self.last_sequence = None
        self.last_warning = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(CompressedImage, topic, qos)
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="front-camera-poller", daemon=True
        )
        self._thread.start()
        self.get_logger().info(
            "direct camera %s -> %s at up to %.1f Hz"
            % (self.url, topic, self.fps)
        )

    def _run(self):
        while not self._stop.is_set() and rclpy.ok():
            started = time.monotonic()
            self.poll()
            remaining = 1.0 / self.fps - (time.monotonic() - started)
            if remaining > 0.0:
                self._stop.wait(remaining)

    def poll(self):
        try:
            with urlopen(self.url, timeout=1.0) as response:
                sequence = response.headers.get("X-Frame-Sequence")
                data = response.read(4 * 1024 * 1024)
            if sequence is not None and sequence == self.last_sequence:
                return
            message = CompressedImage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.frame_id
            message.format = "jpeg"
            message.data = data
            self.publisher.publish(message)
            self.last_sequence = sequence
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_warning >= 10.0:
                self.get_logger().warning("front camera unavailable: %s" % (exc,))
                self.last_warning = now

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--url",
        default=os.environ.get(
            "SCAN_CAMERA_URL", "http://192.168.76.1:8765/frame.jpg"
        ),
    )
    parser.add_argument(
        "--topic", default="/go2/front_camera/image_raw/compressed"
    )
    parser.add_argument("--frame-id", default="go2_front_camera_optical_frame")
    parser.add_argument("--fps", type=float, default=5.0)
    args = parser.parse_args()
    if args.fps <= 0.0:
        parser.error("--fps must be positive")
    rclpy.init()
    node = FrontCameraRelay(args.url, args.topic, args.frame_id, args.fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

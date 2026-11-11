#!/usr/bin/python3
"""Publish calibrated fisheye CameraInfo from the installed calibration YAML."""

import argparse
import math
import sys

import rclpy
import yaml
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


def load_calibration(path):
    with open(path, "r", encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    if calibration.get("schema_version") != 1:
        raise ValueError(f"unsupported calibration schema: {path}")

    camera = calibration["camera"]
    extrinsic = calibration["extrinsic"]
    matrix = camera["camera_matrix"]
    distortion = camera["distortion_coefficients"]
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("camera_matrix must be 3x3")
    if len(distortion) != 4:
        raise ValueError("equidistant CameraInfo requires four distortion values")
    if camera["distortion_model"] != "equidistant":
        raise ValueError("only the calibrated equidistant model is supported")

    quaternion = extrinsic["quaternion_xyzw"]
    norm = math.sqrt(sum(value * value for value in quaternion))
    if abs(norm - 1.0) > 1e-6:
        raise ValueError(f"extrinsic quaternion is not normalized: norm={norm}")
    return calibration


class CameraInfoPublisher(Node):
    def __init__(self, calibration, topic, frame_override, publish_rate):
        super().__init__("go2_front_camera_info")
        camera = calibration["camera"]
        extrinsic = calibration["extrinsic"]
        matrix = camera["camera_matrix"]
        projection = camera["projection_matrix"]

        self.template = CameraInfo()
        self.template.header.frame_id = frame_override or extrinsic["child_frame"]
        self.template.width = int(camera["image_width"])
        self.template.height = int(camera["image_height"])
        self.template.distortion_model = camera["distortion_model"]
        self.template.d = [float(value) for value in camera["distortion_coefficients"]]
        self.template.k = [float(value) for row in matrix for value in row]
        self.template.r = [
            float(value) for row in camera["rectification_matrix"] for value in row
        ]
        self.template.p = [float(value) for row in projection for value in row]

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(CameraInfo, topic, qos)
        self.publish_camera_info()
        self.timer = self.create_timer(max(1.0 / publish_rate, 0.1), self.publish_camera_info)
        self.get_logger().info(
            "publishing %dx%d %s CameraInfo on %s, frame=%s"
            % (
                self.template.width,
                self.template.height,
                self.template.distortion_model,
                topic,
                self.template.header.frame_id,
            )
        )

    def publish_camera_info(self):
        message = CameraInfo()
        message.header = self.template.header
        message.header.stamp = self.get_clock().now().to_msg()
        message.height = self.template.height
        message.width = self.template.width
        message.distortion_model = self.template.distortion_model
        message.d = self.template.d
        message.k = self.template.k
        message.r = self.template.r
        message.p = self.template.p
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration-file", required=True)
    parser.add_argument(
        "--topic", default="/go2/front_camera/camera_info"
    )
    parser.add_argument("--frame-id", default="")
    parser.add_argument("--publish-rate", type=float, default=1.0)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.publish_rate <= 0.0:
        parser.error("--publish-rate must be positive")

    calibration = load_calibration(args.calibration_file)
    rclpy.init()
    node = CameraInfoPublisher(
        calibration, args.topic, args.frame_id, args.publish_rate
    )
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

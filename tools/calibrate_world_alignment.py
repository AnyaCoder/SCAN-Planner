#!/usr/bin/env python3
import argparse
import math
import struct

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2


class AlignmentCheck(Node):
    def __init__(self, odom_topic, cloud_topic):
        super().__init__("world_alignment_check")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom = None
        self.cloud = None
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos)

    def on_odom(self, msg):
        self.odom = msg

    def on_cloud(self, msg):
        self.cloud = msg


def cloud_xyz(msg):
    offsets = {field.name: field.offset for field in msg.fields}
    endian = ">" if msg.is_bigendian else "<"
    points = np.empty((msg.width * msg.height, 3), dtype=np.float64)
    for i in range(points.shape[0]):
        base = i * msg.point_step
        points[i] = (
            struct.unpack_from(endian + "f", msg.data, base + offsets["x"])[0],
            struct.unpack_from(endian + "f", msg.data, base + offsets["y"])[0],
            struct.unpack_from(endian + "f", msg.data, base + offsets["z"])[0],
        )
    return points[np.isfinite(points).all(axis=1)]


def ground_normal(points, center, radius, percentile):
    near = points[np.linalg.norm(points[:, :2] - center[:2], axis=1) < radius]
    if len(near) < 50:
        raise RuntimeError("not enough occupancy points near the robot")
    ground = near[near[:, 2] <= np.percentile(near[:, 2], percentile)]
    centered = ground - ground.mean(axis=0)
    normal = np.linalg.svd(centered, full_matrices=False)[2][-1]
    if normal[2] < 0.0:
        normal = -normal
    return normal / np.linalg.norm(normal), len(ground)


def main():
    parser = argparse.ArgumentParser(
        description="Estimate the world-alignment quaternion from the local ground plane.")
    parser.add_argument("--odom-topic", default="/LIO/odom_imu")
    parser.add_argument("--cloud-topic", default="/grid_map/occupancy")
    parser.add_argument("--radius", type=float, default=1.5)
    parser.add_argument("--ground-percentile", type=float, default=12.0)
    args = parser.parse_args()

    rclpy.init()
    node = AlignmentCheck(args.odom_topic, args.cloud_topic)
    while rclpy.ok() and (node.odom is None or node.cloud is None):
        rclpy.spin_once(node, timeout_sec=0.5)
    if node.odom is None or node.cloud is None:
        raise RuntimeError("timed out waiting for odometry or occupancy")

    pose = node.odom.pose.pose
    center = np.array([pose.position.x, pose.position.y, pose.position.z])
    normal, ground_count = ground_normal(
        cloud_xyz(node.cloud), center, args.radius, args.ground_percentile)
    axis = np.cross(normal, [0.0, 0.0, 1.0])
    angle = math.acos(np.clip(normal[2], -1.0, 1.0))
    if np.linalg.norm(axis) > 1e-9:
        correction = Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle)
        quaternion = correction.as_quat()
    else:
        quaternion = np.array([0.0, 0.0, 0.0, 1.0])

    print(f"ground points: {ground_count}")
    print(f"ground normal: {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}")
    print(f"correction angle: {math.degrees(angle):.3f} deg")
    print("recommended fastlio_bridge.yaml values:")
    print(f"  world_alignment_x: {quaternion[0]:.10f}")
    print(f"  world_alignment_y: {quaternion[1]:.10f}")
    print(f"  world_alignment_z: {quaternion[2]:.10f}")
    print(f"  world_alignment_w: {quaternion[3]:.10f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

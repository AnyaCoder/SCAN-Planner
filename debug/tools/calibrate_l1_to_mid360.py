#!/usr/bin/env python3
"""Estimate a planar L1->MID-360 correction from an overlapping stationary scan."""

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from capture_mid360_topdown import (
    estimate_ground_z,
    odometry_pose,
    sensor_geometry,
    transform_to_world_frame,
)

DEBUG_DIR = Path(__file__).resolve().parents[1]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--range", dest="range_m", type=float, default=5.0)
    parser.add_argument("--min-radius", type=float, default=0.55)
    parser.add_argument("--voxel", type=float, default=0.08)
    parser.add_argument("--max-iterations", type=int, default=40)
    parser.add_argument("--trim-fraction", type=float, default=0.75)
    parser.add_argument(
        "--output-json",
        type=Path,
        default=DEBUG_DIR / "calibration/l1_mid360_calibration.json",
    )
    parser.add_argument(
        "--output-image",
        type=Path,
        default=DEBUG_DIR / "calibration/l1_mid360_calibration.png",
    )
    return parser.parse_args()


def rotation_matrix(yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [[cosine, -sine], [sine, cosine]],
        dtype=np.float64,
    )


def voxel_downsample(points, voxel):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    sums = np.column_stack(
        [
            np.bincount(inverse, weights=points[:, axis])
            for axis in range(points.shape[1])
        ]
    )
    return sums / counts[:, None]


def filtered_xy(points, args):
    xy = np.asarray(points[:, :2], dtype=np.float64)
    z = np.asarray(points[:, 2], dtype=np.float64)
    ground_z = estimate_ground_z(np.asarray(points, dtype=np.float32))
    radius = np.linalg.norm(xy, axis=1)
    mask = (
        np.isfinite(xy).all(axis=1)
        & np.isfinite(z)
        & (radius >= args.min_radius)
        & (radius <= args.range_m)
        & ((z - ground_z) >= 0.12)
        & ((z - ground_z) <= 1.20)
    )
    return voxel_downsample(xy[mask], args.voxel)


def transform_points(points, rotation, translation):
    return points @ rotation.T + translation


def alignment_metrics(source, target_tree, trim_fraction):
    distances, _ = target_tree.query(source)
    count = max(1, int(len(distances) * trim_fraction))
    used = np.partition(distances, count - 1)[:count]
    median = float(np.median(used))
    rmse = float(np.sqrt(np.mean(np.square(used))))
    fit = float(np.mean(distances <= 0.20))
    return {
        "median_m": median,
        "rmse_m": rmse,
        "fit_020": fit,
        "trimmed_points": int(count),
        "cost": median + 0.25 * (1.0 - fit),
    }


def icp_align(source, target, initial_yaw_deg, args):
    target_tree = cKDTree(target)
    rotation = rotation_matrix(math.radians(initial_yaw_deg))
    translation = np.zeros(2, dtype=np.float64)
    source_work = transform_points(source, rotation, translation)
    for _ in range(args.max_iterations):
        distances, indices = target_tree.query(source_work)
        count = max(3, int(len(distances) * args.trim_fraction))
        selected = np.argpartition(distances, count - 1)[:count]
        source_sel = source_work[selected]
        target_sel = target[indices[selected]]
        source_center = source_sel.mean(axis=0)
        target_center = target_sel.mean(axis=0)
        covariance = (
            (source_sel - source_center).T
            @ (target_sel - target_center)
        )
        u, _, vt = np.linalg.svd(covariance)
        delta_rotation = vt.T @ u.T
        if np.linalg.det(delta_rotation) < 0.0:
            vt[-1, :] *= -1.0
            delta_rotation = vt.T @ u.T
        delta_translation = (
            target_center - delta_rotation @ source_center
        )
        source_work = (
            source_work @ delta_rotation.T + delta_translation
        )
        rotation = delta_rotation @ rotation
        translation = delta_rotation @ translation + delta_translation
        if (
            math.degrees(abs(math.atan2(delta_rotation[1, 0], delta_rotation[0, 0])))
            < 0.005
            and np.linalg.norm(delta_translation) < 0.0005
        ):
            break
    metrics = alignment_metrics(
        source_work, target_tree, args.trim_fraction
    )
    return rotation, translation, metrics, source_work


def draw_overlay(reference, source_before, source_after, output, range_m):
    resolution = 0.02
    size = int(round(2.0 * range_m / resolution))
    panels = []
    for source in (source_before, source_after):
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        for points, color in (
            (reference, (255, 130, 40)),
            (source, (70, 70, 255)),
        ):
            valid = (
                (points[:, 0] >= -range_m)
                & (points[:, 0] < range_m)
                & (points[:, 1] >= -range_m)
                & (points[:, 1] < range_m)
            )
            x = np.floor((points[valid, 0] + range_m) / resolution).astype(int)
            y = size - 1 - np.floor(
                (points[valid, 1] + range_m) / resolution
            ).astype(int)
            canvas[y, x] = color
        cv2.circle(
            canvas,
            (size // 2, size // 2),
            int(round(0.55 / resolution)),
            (80, 80, 80),
            1,
            cv2.LINE_AA,
        )
        panels.append(canvas)
    overlay = np.concatenate(panels, axis=1)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), overlay)


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("l1_mid360_calibration")
    cloud_qos = QoSProfile(depth=20)
    cloud_qos.reliability = QoSReliabilityPolicy.RELIABLE
    odom_qos = QoSProfile(depth=100)
    odom_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

    primary_frames = []
    secondary_frames = []
    pose_history = []
    latest_pose = None
    started = None
    wait_started = time.monotonic()

    def primary_callback(msg):
        nonlocal started
        now = time.monotonic()
        if started is None:
            started = now
        if len(msg.points) == 0:
            return
        primary_frames.append(
            {
                "received": now,
                "stamp": float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9,
                "points": np.asarray(
                    [(point.x, point.y, point.z) for point in msg.points],
                    dtype=np.float32,
                ),
            }
        )

    def secondary_callback(msg):
        points = np.asarray(
            list(
                point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            ),
            dtype=np.float32,
        )
        if points.size == 0:
            return
        secondary_frames.append(
            {
                "received": time.monotonic(),
                "points": points.reshape(-1, 3),
            }
        )

    def odom_callback(msg):
        nonlocal latest_pose
        try:
            pose = odometry_pose(msg)
        except ValueError:
            return
        pose["received"] = time.monotonic()
        latest_pose = pose
        pose_history.append(pose)

    subscriptions = [
        node.create_subscription(
            CustomMsg, "/livox/lidar", primary_callback, cloud_qos
        ),
        node.create_subscription(
            PointCloud2,
            "/utlidar/cloud_base",
            secondary_callback,
            cloud_qos,
        ),
        node.create_subscription(
            Odometry,
            "/LIO/odom_vehicle",
            odom_callback,
            odom_qos,
        ),
    ]
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if started is None and now - wait_started > args.timeout:
                raise RuntimeError("timed out waiting for MID-360 frames")
            if started is not None and now - started >= args.duration:
                break
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    if latest_pose is None or len(primary_frames) < 2 or not secondary_frames:
        raise RuntimeError("not enough LiDAR/odometry data for calibration")
    geometry = sensor_geometry()
    imu_to_base = np.linalg.inv(geometry["base_to_imu"])
    lidar_to_base = imu_to_base @ geometry["lidar_to_imu"]
    primary_world = []
    secondary_world = []
    for frame in primary_frames:
        primary_world.append(
            transform_to_world_frame(
                frame["points"],
                lidar_to_base,
                latest_pose,
                pose_history,
                match_time=frame["stamp"],
                pose_time_key="stamp_sec",
            )
        )
    for frame in secondary_frames:
        secondary_world.append(
            transform_to_world_frame(
                frame["points"],
                np.eye(4, dtype=np.float64),
                latest_pose,
                pose_history,
                match_time=frame["received"],
                pose_time_key="received",
                max_time_error=0.20,
            )
        )
    primary_world = np.concatenate(primary_world, axis=0)
    secondary_world = np.concatenate(secondary_world, axis=0)
    current_rotation = np.asarray(
        [
            [math.cos(latest_pose["yaw_rad"]), -math.sin(latest_pose["yaw_rad"])],
            [math.sin(latest_pose["yaw_rad"]), math.cos(latest_pose["yaw_rad"])],
        ]
    )
    current_translation = np.asarray(
        [latest_pose["x"], latest_pose["y"]], dtype=np.float64
    )
    primary_base = np.column_stack(
        (
            primary_world[:, :2] - current_translation,
            primary_world[:, 2],
        )
    )
    primary_base[:, :2] = primary_base[:, :2] @ current_rotation
    secondary_base = np.column_stack(
        (
            secondary_world[:, :2] - current_translation,
            secondary_world[:, 2],
        )
    )
    secondary_base[:, :2] = secondary_base[:, :2] @ current_rotation
    primary_xy = filtered_xy(primary_base, args)
    secondary_xy = filtered_xy(secondary_base, args)
    if len(primary_xy) < 100 or len(secondary_xy) < 100:
        raise RuntimeError("not enough obstacle overlap points for calibration")

    candidates = []
    for yaw_deg in np.arange(-15.0, 15.01, 2.5):
        rotation, translation, metrics, aligned = icp_align(
            secondary_xy, primary_xy, float(yaw_deg), args
        )
        candidates.append(
            {
                "initial_yaw_deg": float(yaw_deg),
                "rotation": rotation,
                "translation": translation,
                "metrics": metrics,
                "aligned": aligned,
            }
        )
    best = min(candidates, key=lambda item: item["metrics"]["cost"])
    rotation = best["rotation"]
    translation = best["translation"]
    yaw_deg = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    target_tree = cKDTree(primary_xy)
    before_metrics = alignment_metrics(
        secondary_xy, target_tree, args.trim_fraction
    )
    result = {
        "secondary_frame": "base",
        "reference_frame": "base",
        "correction": {
            "x_m": float(translation[0]),
            "y_m": float(translation[1]),
            "yaw_deg": float(yaw_deg),
        },
        "before": before_metrics,
        "after": best["metrics"],
        "points": {
            "mid360_obstacles": int(len(primary_xy)),
            "l1_obstacles": int(len(secondary_xy)),
            "voxel_m": args.voxel,
        },
        "initial_yaw_deg": best["initial_yaw_deg"],
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    draw_overlay(
        primary_xy, secondary_xy, best["aligned"], args.output_image, args.range_m
    )
    print(json.dumps(result, indent=2), flush=True)
    print("overlay:", args.output_image, flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Capture a dual-LiDAR window and render an on-demand top-down PNG."""

import argparse
import json
import math
import os
import re
import time
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from PIL.PngImagePlugin import PngInfo


GO2_BASE_TO_IMU_XYZ = np.array([-0.02557, 0.0, 0.04232], dtype=np.float64)
GO2_BASE_TO_L1_XYZ = np.array([0.28945, 0.0, -0.046825], dtype=np.float64)
GO2_BASE_TO_CAMERA_XYZ = np.array([0.32715, -0.00003, 0.04297], dtype=np.float64)
# Measured from the stationary MID-360/L1 overlap. This corrects residual
# planar error in Unitree's published cloud_base frame without touching the
# already accurate MID-360 frame.
L1_BASE_CORRECTION_XYZ = np.array(
    [-0.2337208605, 0.1013372867, 0.0],
    dtype=np.float64,
)
L1_BASE_CORRECTION_YAW_DEG = -1.969417957
OBSTACLE_MIN_HEIGHT_M = 0.12
# The MID-360 also sees the Go2 body just beyond the physical footprint. Leave
# a small self-return guard band for navigation rays while keeping the SCAN
# planner's configured footprint unchanged in the rendered map.
SELF_RETURN_CLEARANCE_M = 0.10

# FAST-LIO stores the MID-360 LiDAR extrinsic as LiDAR_T_wrt_IMU.
MID360_LIDAR_TO_BODY_XYZ = np.array([-0.011, -0.02329, 0.04412], dtype=np.float64)

# Calibrated camera-optical -> FAST-LIO body transform from
# src/go2_camera_calibration/config/go2_front_camera.yaml.
BODY_TO_CAMERA_OPTICAL = np.array(
    [
        [0.0061053243, 0.2455715267, 0.9693592473, 0.3769035887],
        [-0.9995917241, 0.0285570277, -0.0009387245, -0.0229584961],
        [-0.0279125429, -0.9689577501, 0.2456456157, 0.0462013547],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture Livox MID-360 CustomMsg data and render a 5 cm top-down map."
    )
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument(
        "--secondary-topic",
        default="/utlidar/cloud_base",
        help="Go2 native LiDAR cloud already transformed into base_link.",
    )
    parser.add_argument(
        "--odom-topic",
        default="/LIO/odom_vehicle",
        help="World-frame Go2 base odometry recorded with the snapshot.",
    )
    parser.add_argument(
        "--camera-url",
        default="http://127.0.0.1:8765/frame.jpg",
        help="Camera endpoint recorded in snapshot metadata.",
    )
    parser.add_argument(
        "--snapshot-sequence",
        type=int,
        default=0,
        help="Waypoint snapshot sequence recorded in snapshot metadata.",
    )
    parser.add_argument(
        "--no-secondary",
        action="store_true",
        help="Capture MID-360 only instead of fusing the Go2 native LiDAR.",
    )
    parser.add_argument(
        "--planner-config",
        type=Path,
        default=(
            Path(__file__).resolve().parents[1]
            / "src/planner/plan_manage/config/planner.yaml"
        ),
        help="SCAN-Planner YAML containing the double-cylinder footprint.",
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument(
        "--rolling",
        action="store_true",
        help="Keep the latest --duration second point-cloud window in memory.",
    )
    parser.add_argument(
        "--trigger-file",
        type=Path,
        help="Render one top-down image when this request file exists.",
    )
    parser.add_argument(
        "--ready-file",
        type=Path,
        help="Write this status file once when the rolling window is ready.",
    )
    parser.add_argument("--resolution", type=float, default=0.05, help="Map cell size in metres.")
    parser.add_argument("--range", dest="range_m", type=float, default=5.0)
    parser.add_argument(
        "--style",
        choices=("green_points", "relief"),
        default="green_points",
        help="Raw point-cloud projection or the smoothed height-relief renderer.",
    )
    parser.add_argument(
        "--projection-resolution",
        type=float,
        default=0.01,
        help="Display pixel size for the green point-cloud projection.",
    )
    parser.add_argument(
        "--max-height",
        type=float,
        default=1.20,
        help="Ignore returns above this height above the floor, including the ceiling.",
    )
    parser.add_argument("--pixels-per-cell", type=int, default=6)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            Path(__file__).resolve().parents[2]
            / "debug/pointcloud_views/mid360_topdown_5s.png"
        ),
    )
    parser.add_argument("--input-pcd", type=Path, help="Render a saved PCD instead of capturing ROS data.")
    parser.add_argument("--no-raw", action="store_true", help="Do not keep the captured NPZ data.")
    return parser.parse_args()


def load_pcd_xyz(path):
    fields = []
    sizes = []
    types = []
    counts = []
    data_kind = ""
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD header has no DATA line: %s" % path)
            text = line.decode("ascii", "replace").strip()
            parts = text.split()
            if not parts:
                continue
            if parts[0] == "FIELDS":
                fields = parts[1:]
            elif parts[0] == "SIZE":
                sizes = [int(value) for value in parts[1:]]
            elif parts[0] == "TYPE":
                types = parts[1:]
            elif parts[0] == "COUNT":
                counts = [int(value) for value in parts[1:]]
            elif parts[0] == "DATA":
                data_kind = parts[1].lower()
                break
        if data_kind != "binary":
            raise RuntimeError("Only binary PCD files are supported: %s" % path)
        raw = np.fromfile(stream, dtype=np.uint8)

    if len(counts) != len(fields):
        counts = [1] * len(fields)
    point_step = sum(size * count for size, count in zip(sizes, counts))
    if point_step == 0 or raw.size % point_step:
        raise RuntimeError("Invalid PCD point stride in %s" % path)
    rows = raw.reshape(-1, point_step)
    offsets = np.cumsum([0] + [size * count for size, count in zip(sizes, counts)])
    columns = {}
    for index, name in enumerate(fields):
        if name not in ("x", "y", "z"):
            continue
        size = sizes[index]
        dtype_code = {"F": "f", "I": "i", "U": "u"}.get(types[index])
        if dtype_code is None or size not in (4, 8):
            raise RuntimeError("Unsupported PCD field %s in %s" % (name, path))
        values = rows[:, offsets[index] : offsets[index + 1]].copy()
        values = values.view("<%s%d" % (dtype_code, size)).reshape(len(rows), -1)[:, 0]
        columns[name] = values.astype(np.float32)
    if not all(name in columns for name in ("x", "y", "z")):
        raise RuntimeError("PCD must contain x, y and z fields: %s" % path)
    return np.column_stack((columns["x"], columns["y"], columns["z"]))


def load_font(size, bold=False):
    names = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def load_dual_cylinder_model(path):
    if not path.is_file():
        raise RuntimeError("SCAN-Planner config not found: %s" % path)
    text = path.read_text(encoding="utf-8")

    def scalar(name):
        match = re.search(
            r"^\s*%s:\s*([-+0-9.eE]+)\s*$" % re.escape(name),
            text,
            flags=re.MULTILINE,
        )
        if match is None:
            raise RuntimeError("Missing %s in %s" % (name, path))
        return float(match.group(1))

    radius = scalar("grid_map.double_cylinder_radius")
    offset = scalar("grid_map.double_cylinder_offset")
    resolution = scalar("grid_map.resolution")
    if radius <= 0.0 or offset < 0.0 or resolution <= 0.0:
        raise RuntimeError("Invalid SCAN-Planner double-cylinder parameters")
    return radius, offset, resolution


def rigid_transform(rotation=None, translation=None):
    transform = np.eye(4, dtype=np.float64)
    if rotation is not None:
        transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    if translation is not None:
        transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def sensor_geometry():
    # Go2 front camera optical axes follow ROS: +x right, +y down, +z forward.
    base_to_camera_optical_rotation = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=np.float64,
    )
    base_to_camera = rigid_transform(
        base_to_camera_optical_rotation, GO2_BASE_TO_CAMERA_XYZ
    )
    base_to_body = base_to_camera @ np.linalg.inv(BODY_TO_CAMERA_OPTICAL)
    base_to_imu = rigid_transform(
        translation=-GO2_BASE_TO_IMU_XYZ
    )
    imu_to_base = np.linalg.inv(base_to_imu)
    body_to_lidar = rigid_transform(translation=MID360_LIDAR_TO_BODY_XYZ)
    # With the transform convention used by transform_xyz(), this matrix maps
    # MID-360 LiDAR points into the Go2 IMU frame.
    lidar_to_imu = base_to_imu @ base_to_body @ body_to_lidar
    mid360_position_imu = (base_to_imu @ base_to_body)[:3, 3]
    l1_position_imu = GO2_BASE_TO_L1_XYZ - GO2_BASE_TO_IMU_XYZ
    l1_to_base = rigid_transform(
        rotation=np.array(
            [
                [
                    math.cos(math.radians(L1_BASE_CORRECTION_YAW_DEG)),
                    -math.sin(math.radians(L1_BASE_CORRECTION_YAW_DEG)),
                    0.0,
                ],
                [
                    math.sin(math.radians(L1_BASE_CORRECTION_YAW_DEG)),
                    math.cos(math.radians(L1_BASE_CORRECTION_YAW_DEG)),
                    0.0,
                ],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        translation=L1_BASE_CORRECTION_XYZ,
    )
    return {
        "lidar_to_imu": lidar_to_imu,
        "l1_to_base": l1_to_base,
        "base_to_imu": base_to_imu,
        "imu_to_base": imu_to_base,
        "base_to_body": base_to_body,
        "imu_position_base": GO2_BASE_TO_IMU_XYZ,
        "mid360_position_imu": mid360_position_imu,
        "l1_position_imu": l1_position_imu,
        "l1_base_correction_xyz": L1_BASE_CORRECTION_XYZ,
        "l1_base_correction_yaw_deg": L1_BASE_CORRECTION_YAW_DEG,
    }


def transform_xyz(points, transform):
    xyz = np.asarray(points[:, :3], dtype=np.float64)
    transformed = xyz @ transform[:3, :3].T + transform[:3, 3]
    result = np.asarray(points, dtype=np.float32).copy()
    result[:, :3] = transformed.astype(np.float32)
    return result


def odometry_pose(msg):
    pose = msg.pose.pose
    qx = float(pose.orientation.x)
    qy = float(pose.orientation.y)
    qz = float(pose.orientation.z)
    qw = float(pose.orientation.w)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        raise ValueError("odometry quaternion is zero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    velocity = msg.twist.twist.linear
    return {
        "stamp_sec": float(msg.header.stamp.sec)
        + float(msg.header.stamp.nanosec) * 1e-9,
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
        "roll_rad": roll,
        "pitch_rad": pitch,
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
        "speed_mps": math.sqrt(
            float(velocity.x) ** 2
            + float(velocity.y) ** 2
            + float(velocity.z) ** 2
        ),
    }


def compute_dual_cylinder_clearance(
    x,
    y,
    relative_z,
    radius,
    offset,
    resolution,
    range_m,
    max_height,
):
    obstacle = (
        (relative_z >= OBSTACLE_MIN_HEIGHT_M)
        & (relative_z <= max_height)
        & (x >= -range_m)
        & (x < range_m)
        & (y >= -range_m)
        & (y < range_m)
    )
    if not np.any(obstacle):
        return {
            "front_m": float(range_m - offset - radius),
            "rear_m": float(range_m - offset - radius),
            "nearest_m": float(range_m - offset - radius),
            "nearest_center": None,
            "nearest_point": None,
        }

    # Match the planner's 5 cm voxel centres before evaluating the two
    # circular cylinder footprints.
    ix = np.floor((x[obstacle] + range_m) / resolution).astype(np.int64)
    iy = np.floor((y[obstacle] + range_m) / resolution).astype(np.int64)
    cell_ids = np.unique(iy * int(round(2.0 * range_m / resolution)) + ix)
    cells_per_axis = int(round(2.0 * range_m / resolution))
    obstacle_xy = np.column_stack(
        (
            (cell_ids % cells_per_axis + 0.5) * resolution - range_m,
            (cell_ids // cells_per_axis + 0.5) * resolution - range_m,
        )
    )

    centers = {
        "front": np.array([offset, 0.0]),
        "rear": np.array([-offset, 0.0]),
    }
    distances_by_center = {
        name: np.linalg.norm(obstacle_xy - center, axis=1)
        for name, center in centers.items()
    }
    # Ignore robot self-returns inside the union of the two cylinders. A point
    # inside the other cylinder is still part of the robot footprint.
    outside_hull = np.minimum(
        distances_by_center["front"], distances_by_center["rear"]
    ) >= radius
    if not np.any(outside_hull):
        return {
            "front_m": 0.0,
            "rear_m": 0.0,
            "nearest_m": 0.0,
            "nearest_center": None,
            "nearest_point": None,
        }

    cylinder_results = {}
    for name, center in centers.items():
        distances = distances_by_center[name]
        signed_clearance = distances - radius
        external = outside_hull
        if not np.any(external):
            cylinder_results[name] = {
                "clearance": 0.0,
                "center": center,
                "point": center,
            }
            continue
        external_indices = np.flatnonzero(external)
        index = external_indices[np.argmin(signed_clearance[external])]
        cylinder_results[name] = {
            "clearance": float(signed_clearance[index]),
            "center": center,
            "point": obstacle_xy[index],
        }

    nearest_name = min(
        cylinder_results, key=lambda name: cylinder_results[name]["clearance"]
    )
    nearest = cylinder_results[nearest_name]
    return {
        "front_m": cylinder_results["front"]["clearance"],
        "rear_m": cylinder_results["rear"]["clearance"],
        "nearest_m": nearest["clearance"],
        "nearest_center": nearest["center"],
        "nearest_point": nearest["point"],
    }


def _direction_label(angle_deg):
    labels = (
        "front",
        "front-left",
        "left",
        "rear-left",
        "rear",
        "rear-right",
        "right",
        "front-right",
    )
    index = int(round((angle_deg % 360.0) / 45.0)) % len(labels)
    return labels[index]


def _open_angle_clusters(open_mask):
    if np.all(open_mask):
        return [np.arange(len(open_mask))]
    if not np.any(open_mask):
        return []

    clusters = []
    current = []
    for index, is_open in enumerate(open_mask):
        if is_open:
            current.append(index)
        elif current:
            clusters.append(np.asarray(current, dtype=np.int64))
            current = []
    if current:
        clusters.append(np.asarray(current, dtype=np.int64))
    if len(clusters) > 1 and open_mask[0] and open_mask[-1]:
        tail = clusters.pop()
        head = clusters.pop(0)
        clusters.insert(0, np.concatenate((tail, head + len(open_mask))))
    return clusters


def analyse_traversable_directions(
    x,
    y,
    relative_z,
    radius,
    offset,
    resolution,
    range_m,
    max_height,
):
    cells = int(round(2.0 * range_m / resolution))
    ix = np.floor((x + range_m) / resolution).astype(np.int64)
    iy = np.floor((y + range_m) / resolution).astype(np.int64)
    obstacle_points = (
        (relative_z >= OBSTACLE_MIN_HEIGHT_M)
        & (relative_z <= max_height)
        & (ix >= 0)
        & (ix < cells)
        & (iy >= 0)
        & (iy < cells)
    )
    occupancy = np.zeros((cells, cells), dtype=bool)
    occupancy[iy[obstacle_points], ix[obstacle_points]] = True

    # Remove returns on the robot's current footprint. They are sensor
    # self-returns rather than environment and would block every candidate ray.
    cell_centres_x = (np.arange(cells) + 0.5) * resolution - range_m
    cell_centres_y = (np.arange(cells) + 0.5) * resolution - range_m
    grid_x, grid_y = np.meshgrid(cell_centres_x, cell_centres_y)
    inside_current_hull = np.minimum(
        np.hypot(grid_x - offset, grid_y),
        np.hypot(grid_x + offset, grid_y),
    ) < radius + SELF_RETURN_CLEARANCE_M
    occupancy[inside_current_hull] = False

    offsets = []
    steps = int(math.ceil(radius / resolution))
    for offset_y in range(-steps, steps + 1):
        for offset_x in range(-steps, steps + 1):
            if math.hypot(offset_x * resolution, offset_y * resolution) <= radius + resolution:
                offsets.append((offset_x, offset_y))
    padded_occupancy = np.pad(occupancy, steps, mode="constant")

    def footprint_blocked(center_x, center_y):
        center_ix = (
            np.floor((center_x + range_m) / resolution).astype(np.int64)
            + steps
        )
        center_iy = (
            np.floor((center_y + range_m) / resolution).astype(np.int64)
            + steps
        )
        blocked = np.zeros_like(center_ix, dtype=bool)
        for offset_x, offset_y in offsets:
            blocked |= padded_occupancy[
                center_iy + offset_y,
                center_ix + offset_x,
            ]
        return blocked

    angles_deg = np.arange(0.0, 360.0, 10.0)
    angles_rad = np.deg2rad(angles_deg)
    free_distance = np.zeros(len(angles_deg), dtype=np.float64)
    max_path = max(0.0, range_m - radius - 2.0 * resolution)
    step_size = resolution / 2.0
    active = np.ones(len(angles_deg), dtype=bool)
    for distance in np.arange(step_size, max_path + 1e-9, step_size):
        active_indices = np.flatnonzero(active)
        if active_indices.size == 0:
            break
        direction_x = np.cos(angles_rad[active_indices])
        direction_y = np.sin(angles_rad[active_indices])
        front_center_x = (distance + offset) * direction_x
        front_center_y = (distance + offset) * direction_y
        rear_center_x = (distance - offset) * direction_x
        rear_center_y = (distance - offset) * direction_y
        blocked = footprint_blocked(front_center_x, front_center_y)
        blocked |= footprint_blocked(rear_center_x, rear_center_y)
        free_distance[active_indices[~blocked]] = distance
        active[active_indices[blocked]] = False
    free_distance[active] = max_path

    open_mask = free_distance >= 0.75
    clusters = _open_angle_clusters(open_mask)
    candidates = []
    for cluster in clusters:
        unwrapped_deg = angles_deg[cluster % len(angles_deg)]
        local_index = int(np.argmax(free_distance[cluster % len(angles_deg)]))
        angle_deg = float(unwrapped_deg[local_index] % 360.0)
        candidates.append(
            {
                "angle_deg": angle_deg,
                "direction": _direction_label(angle_deg),
                "distance_m": float(
                    free_distance[cluster[local_index] % len(angles_deg)]
                ),
            }
        )
    candidates.sort(key=lambda item: item["distance_m"], reverse=True)

    def distance_at(angle_deg):
        index = int(round((angle_deg % 360.0) / 10.0)) % len(angles_deg)
        return float(free_distance[index])

    return {
        "angles_deg": angles_deg,
        "free_distance_m": free_distance,
        "open_mask": open_mask,
        "open_sectors": len(clusters),
        "candidates": candidates,
        "forward_m": distance_at(0.0),
        "left_m": distance_at(90.0),
        "right_m": distance_at(270.0),
    }


def pose_rotation_matrix(pose):
    qx = float(pose["qx"])
    qy = float(pose["qy"])
    qz = float(pose["qz"])
    qw = float(pose["qw"])
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_to_world_frame(
    points,
    to_base,
    current_pose,
    pose_history,
    match_time=None,
    pose_time_key="stamp_sec",
    max_time_error=0.5,
):
    xyz = np.asarray(points[:, :3], dtype=np.float64)
    xyz = xyz @ to_base[:3, :3].T + to_base[:3, 3]
    if current_pose is not None:
        frame_pose = current_pose
        if match_time is not None and pose_history:
            nearest = min(
                pose_history,
                key=lambda pose: abs(
                    float(pose[pose_time_key]) - float(match_time)
                ),
            )
            if (
                abs(float(nearest[pose_time_key]) - float(match_time))
                <= max_time_error
            ):
                frame_pose = nearest
        frame_rotation = pose_rotation_matrix(frame_pose)
        frame_translation = np.array(
            [frame_pose["x"], frame_pose["y"], frame_pose["z"]],
            dtype=np.float64,
        )
        xyz = xyz @ frame_rotation.T + frame_translation
    return xyz


def world_to_centered_world_map(points, current_pose, base_to_imu):
    xyz = np.asarray(points[:, :3], dtype=np.float64)
    if current_pose is not None:
        current_rotation = pose_rotation_matrix(current_pose)
        base_translation = np.array(
            [current_pose["x"], current_pose["y"], current_pose["z"]],
            dtype=np.float64,
        )
        imu_origin_world = (
            base_translation + current_rotation @ base_to_imu[:3, 3]
        )
        xyz = xyz - imu_origin_world
    else:
        imu_origin_world = np.zeros(3, dtype=np.float64)
    return xyz, imu_origin_world


def write_json_atomic(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def run_rolling_topdown(
    args,
    cylinder_radius,
    cylinder_offset,
    map_resolution,
    geometry,
):
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    if args.trigger_file is None:
        raise SystemExit("--trigger-file is required in rolling mode")

    rclpy.init()
    node = rclpy.create_node("mid360_topdown_rolling")
    cloud_qos = QoSProfile(depth=10)
    cloud_qos.reliability = QoSReliabilityPolicy.RELIABLE
    odom_qos = QoSProfile(depth=20)
    odom_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

    primary_frames = deque()
    secondary_frames = deque()
    pose_history = deque(maxlen=400)
    latest_pose = None
    wait_started = time.monotonic()
    rolling_started = None
    ready = False
    render_count = 0

    def primary_callback(msg):
        nonlocal rolling_started
        if len(msg.points) == 0:
            return
        received = time.monotonic()
        if rolling_started is None:
            rolling_started = received
        points = np.asarray(
            [(point.x, point.y, point.z) for point in msg.points],
            dtype=np.float32,
        )
        primary_frames.append(
            {
                "received": received,
                "stamp": float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9,
                "points": points,
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
                "stamp": float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9,
                "points": points.reshape(-1, 3),
            }
        )

    def odom_callback(msg):
        nonlocal latest_pose
        received = time.monotonic()
        try:
            pose = odometry_pose(msg)
        except ValueError:
            return
        pose["received"] = received
        latest_pose = pose
        pose_history.append(pose)

    primary_subscription = node.create_subscription(
        CustomMsg, args.topic, primary_callback, cloud_qos
    )
    secondary_subscription = None
    if not args.no_secondary:
        secondary_subscription = node.create_subscription(
            PointCloud2,
            args.secondary_topic,
            secondary_callback,
            cloud_qos,
        )
    odom_subscription = node.create_subscription(
        Odometry,
        args.odom_topic,
        odom_callback,
        odom_qos,
    )
    imu_to_base = np.linalg.inv(geometry["base_to_imu"])
    lidar_to_base = imu_to_base @ geometry["lidar_to_imu"]
    output_path = args.output
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now_monotonic = time.monotonic()
            if not primary_frames:
                if now_monotonic - wait_started > args.timeout:
                    raise RuntimeError(
                        "No MID-360 messages received within %.1f seconds"
                        % args.timeout
                    )
                continue
            cutoff = now_monotonic - args.duration
            while primary_frames and primary_frames[0]["received"] < cutoff:
                primary_frames.popleft()
            while (
                secondary_frames
                and secondary_frames[0]["received"] < cutoff
            ):
                secondary_frames.popleft()
            if not ready:
                window_age = now_monotonic - rolling_started
                if window_age < args.duration:
                    continue
                ready = True
                if args.ready_file is not None:
                    write_json_atomic(
                        args.ready_file,
                        {
                            "rolling_ready": True,
                            "rolling_window_s": args.duration,
                            "window_oldest_age_s": round(window_age, 3),
                            "primary_frames": len(primary_frames),
                            "secondary_frames": len(secondary_frames),
                            "rendered_on_demand": True,
                            "trigger_file": str(args.trigger_file),
                        },
                    )
                print(
                    "rolling point-cloud window ready: %.1f s, "
                    "%d MID-360 frames, %d Go2 LiDAR frames"
                    % (
                        window_age,
                        len(primary_frames),
                        len(secondary_frames),
                    ),
                    flush=True,
                )

            try:
                args.trigger_file.unlink()
            except FileNotFoundError:
                continue
            except OSError as exc:
                print(
                    "cannot consume capture trigger %s: %s"
                    % (args.trigger_file, exc),
                    flush=True,
                )
                continue

            render_count += 1
            requested_at_unix = time.time()
            now_monotonic = time.monotonic()
            transform_started = time.monotonic()
            fused_world = []
            primary_times = []
            primary_counts = []
            for frame in primary_frames:
                fused_world.append(
                    transform_to_world_frame(
                        frame["points"],
                        lidar_to_base,
                        latest_pose,
                        pose_history,
                        match_time=frame["stamp"],
                        pose_time_key="stamp_sec",
                    )
                )
                primary_times.append(frame["received"])
                primary_counts.append(len(frame["points"]))

            fused_secondary = []
            secondary_times = []
            secondary_counts = []
            for frame in secondary_frames:
                fused_secondary.append(
                    transform_to_world_frame(
                        frame["points"],
                        geometry["l1_to_base"],
                        latest_pose,
                        pose_history,
                        # The Go2 native LiDAR header clock is not in the same
                        # time domain as the FAST-LIO odometry header. Pair it
                        # with odometry by local arrival time instead.
                        match_time=frame["received"],
                        pose_time_key="received",
                        max_time_error=0.20,
                    )
                )
                secondary_times.append(frame["received"])
                secondary_counts.append(len(frame["points"]))

            fused_world = np.concatenate(fused_world, axis=0)
            source_labels = np.zeros(len(fused_world), dtype=np.uint8)
            secondary_points = np.empty((0, 3), dtype=np.float32)
            if fused_secondary:
                secondary_points = np.concatenate(
                    fused_secondary, axis=0
                ).astype(np.float32)
                fused_world = np.concatenate(
                    (fused_world, secondary_points), axis=0
                )
                source_labels = np.concatenate(
                    (
                        source_labels,
                        np.ones(len(secondary_points), dtype=np.uint8),
                    )
                )
            # Fuse both LiDARs in the SCAN world frame, then recenter the map
            # on the current Go2 IMU without rotating world axes. The dog arrow
            # rotates with odometry while the map direction remains fixed.
            points, imu_origin_world = world_to_centered_world_map(
                fused_world,
                latest_pose,
                geometry["base_to_imu"],
            )
            points = points.astype(np.float32)
            transform_elapsed = time.monotonic() - transform_started

            temporary_output = output_path.with_name(
                output_path.stem + ".tmp" + output_path.suffix
            )
            stats = render_green_points_topdown(
                points,
                primary_times,
                primary_counts,
                args.range_m,
                args.max_height,
                args.projection_resolution,
                temporary_output,
                geometry,
                cylinder_radius,
                cylinder_offset,
                map_resolution,
                map_heading_rad=(
                    0.0 if latest_pose is None else latest_pose["yaw_rad"]
                ),
                source_labels=source_labels,
                secondary_frame_count=len(fused_secondary),
                secondary_points_captured=len(secondary_points),
            )
            os.replace(temporary_output, output_path)
            stats.update(
                {
                    "output": str(output_path),
                    "rolling": False,
                    "rendered_on_demand": True,
                    "rolling_window_s": args.duration,
                    "rolling_ready": True,
                    "generated_at_unix": requested_at_unix,
                    "render_count": render_count,
                    "transform_elapsed_s": round(transform_elapsed, 3),
                    "render_elapsed_s": round(
                        time.monotonic() - transform_started - transform_elapsed,
                        3,
                    ),
                    "window_oldest_age_s": round(
                        now_monotonic
                        - min(frame["received"] for frame in primary_frames),
                        3,
                    ),
                    "window_newest_age_s": round(
                        now_monotonic
                        - max(frame["received"] for frame in primary_frames),
                        3,
                    ),
                    "odometry_frames": len(pose_history),
                    "secondary_time_source": "receive_monotonic",
                    "secondary_pose_match_window_s": 0.20,
                    "robot_pose": latest_pose,
                    "robot_pose_available": latest_pose is not None,
                    "map_orientation": "world_fixed",
                    "map_origin_world_xyz_m": (
                        imu_origin_world.astype(float).tolist()
                    ),
                    "camera_url": args.camera_url,
                    "snapshot_sequence": 0,
                }
            )
            write_json_atomic(output_path.with_suffix(".json"), stats)
            print(
                "rendered on-demand %.1fs window: %d frames"
                % (
                    args.duration,
                    len(primary_frames),
                ),
                flush=True,
            )
    finally:
        node.destroy_subscription(primary_subscription)
        if secondary_subscription is not None:
            node.destroy_subscription(secondary_subscription)
        node.destroy_subscription(odom_subscription)
        node.destroy_node()
        rclpy.shutdown()


def collect_cloud(topic, duration, timeout, secondary_topic=None, odom_topic=None):
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = rclpy.create_node("mid360_topdown_capture")
    qos = QoSProfile(depth=10)
    qos.reliability = QoSReliabilityPolicy.RELIABLE

    chunks = []
    frame_times = []
    point_counts = []
    secondary_chunks = []
    secondary_frame_times = []
    secondary_point_counts = []
    latest_pose = None
    started = None
    wait_started = time.monotonic()

    def callback(msg):
        nonlocal started
        now = time.monotonic()
        if started is None:
            started = now
        if len(msg.points) == 0:
            return
        points = np.asarray(
            [(p.x, p.y, p.z, p.reflectivity) for p in msg.points],
            dtype=np.float32,
        )
        chunks.append(points)
        frame_times.append(float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9)
        point_counts.append(len(points))

    def secondary_callback(msg):
        if started is None:
            return
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
        points = points.reshape(-1, 3)
        secondary_chunks.append(points)
        secondary_frame_times.append(
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        )
        secondary_point_counts.append(len(points))

    def odom_callback(msg):
        nonlocal latest_pose
        try:
            latest_pose = odometry_pose(msg)
        except ValueError:
            return

    subscription = node.create_subscription(CustomMsg, topic, callback, qos)
    secondary_subscription = None
    if secondary_topic:
        secondary_subscription = node.create_subscription(
            PointCloud2, secondary_topic, secondary_callback, qos
        )
    odom_qos = QoSProfile(depth=5)
    odom_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
    odom_subscription = None
    if odom_topic:
        odom_subscription = node.create_subscription(
            Odometry, odom_topic, odom_callback, odom_qos
        )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if started is None and now - wait_started > timeout:
                raise RuntimeError("No MID-360 messages received within %.1f seconds" % timeout)
            if started is not None and now - started >= duration:
                break
    finally:
        node.destroy_subscription(subscription)
        if secondary_subscription is not None:
            node.destroy_subscription(secondary_subscription)
        if odom_subscription is not None:
            node.destroy_subscription(odom_subscription)
        node.destroy_node()
        rclpy.shutdown()

    if not chunks:
        raise RuntimeError("MID-360 produced no non-empty frames")
    points = np.concatenate(chunks, axis=0)
    secondary_points = (
        np.concatenate(secondary_chunks, axis=0) if secondary_chunks else np.empty((0, 3))
    )
    return (
        points,
        frame_times,
        point_counts,
        secondary_points,
        secondary_frame_times,
        secondary_point_counts,
        latest_pose,
    )


def estimate_ground_z(points):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    near = (x * x + y * y < 16.0) & (z > -1.5) & (z < 1.0)
    candidates = z[near]
    if candidates.size < 100:
        candidates = z[(z > -1.5) & (z < 1.0)]
    if candidates.size == 0:
        return float(np.percentile(z, 5))

    # The robot body and its surrounding structure can produce far more
    # returns than the floor does. Anchor the search to the low tail of the
    # height distribution and then take the densest local mode.
    anchor = float(np.percentile(candidates, 5))
    lower = max(-1.5, anchor - 0.20)
    upper = min(0.8, anchor + 0.15)
    edges = np.arange(lower, upper + 0.0001, 0.02)
    if edges.size < 3:
        return anchor
    hist, edges = np.histogram(candidates, bins=edges)
    smooth = np.convolve(hist, np.ones(3, dtype=np.int64), mode="same")
    index = int(np.argmax(smooth))
    if int(hist[index]) < 100:
        return anchor
    return float((edges[index] + edges[index + 1]) * 0.5)


def gaussian_blur_2d(values, sigma):
    if sigma <= 0:
        return values.astype(np.float32)
    radius = max(1, int(math.ceil(3.0 * sigma)))
    offsets = np.arange(-radius, radius + 1, dtype=np.float32)
    kernel = np.exp(-0.5 * (offsets / sigma) ** 2)
    kernel /= kernel.sum()
    result = values.astype(np.float32)
    for axis in (0, 1):
        pad = [(0, 0), (0, 0)]
        pad[axis] = (radius, radius)
        padded = np.pad(result, pad, mode="constant")
        blurred = np.zeros_like(result)
        for index, weight in enumerate(kernel):
            if axis == 0:
                blurred += weight * padded[index : index + result.shape[0], :]
            else:
                blurred += weight * padded[:, index : index + result.shape[1]]
        result = blurred
    return result


def height_colours(height_fraction):
    stops = np.asarray([0.0, 0.36, 0.72, 1.0], dtype=np.float32)
    colours = np.asarray(
        [
            [66, 205, 222],
            [100, 214, 153],
            [247, 190, 83],
            [242, 103, 116],
        ],
        dtype=np.float32,
    )
    clipped = np.clip(height_fraction, 0.0, 1.0)
    return np.stack(
        [np.interp(clipped, stops, colours[:, channel]) for channel in range(3)],
        axis=-1,
    )


def render_green_points_topdown(
    points,
    frame_times,
    point_counts,
    range_m,
    max_height,
    projection_resolution,
    output,
    geometry,
    cylinder_radius,
    cylinder_offset,
    map_resolution,
    map_heading_rad=0.0,
    source_labels=None,
    secondary_frame_count=0,
    secondary_points_captured=0,
):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    in_range = finite & (x >= -range_m) & (x < range_m) & (y >= -range_m) & (y < range_m)
    if source_labels is not None:
        source_labels_in_range = source_labels[in_range]
    else:
        source_labels_in_range = None
    x, y, z = x[in_range], y[in_range], z[in_range]
    if x.size == 0:
        raise RuntimeError("No points fall inside the requested top-down range")

    ground_points = np.column_stack((x, y, z))
    if source_labels_in_range is not None and np.count_nonzero(source_labels_in_range) >= 100:
        # The native Go2 LiDAR sees clean floor around the blind spot without
        # the MID-360's high-density body returns at z=0.
        ground_points = ground_points[source_labels_in_range == 1]
    ground_z = estimate_ground_z(ground_points)
    relative_z = z - ground_z
    height_mask = (relative_z >= -0.08) & (relative_z <= max_height)
    x, y, relative_z = x[height_mask], y[height_mask], relative_z[height_mask]
    if x.size == 0:
        raise RuntimeError("No points remain after removing the ceiling")
    # Planner-facing direction checks use dog-relative axes. Rendering below
    # still uses the original world-fixed x/y coordinates.
    navigation_cosine = math.cos(float(map_heading_rad))
    navigation_sine = math.sin(float(map_heading_rad))
    navigation_x = navigation_cosine * x + navigation_sine * y
    navigation_y = -navigation_sine * x + navigation_cosine * y
    clearance = compute_dual_cylinder_clearance(
        navigation_x,
        navigation_y,
        relative_z,
        cylinder_radius,
        cylinder_offset,
        map_resolution,
        range_m,
        max_height,
    )
    navigation = analyse_traversable_directions(
        navigation_x,
        navigation_y,
        relative_z,
        cylinder_radius,
        cylinder_offset,
        map_resolution,
        range_m,
        max_height,
    )

    if source_labels is not None:
        source_labels = source_labels_in_range[height_mask]
    ground_mask = relative_z < OBSTACLE_MIN_HEIGHT_M

    cells = max(1, int(round(2.0 * range_m / projection_resolution)))
    ix = np.floor((x + range_m) / projection_resolution).astype(np.int32)
    iy = np.floor((y + range_m) / projection_resolution).astype(np.int32)
    valid = (ix >= 0) & (ix < cells) & (iy >= 0) & (iy < cells)
    ids = (iy[valid] * cells + ix[valid]).astype(np.int64)
    ground_valid = ground_mask[valid]
    obstacle_valid = ~ground_valid
    if source_labels is None:
        primary_obstacle_counts = np.bincount(
            ids[obstacle_valid], minlength=cells * cells
        )
        secondary_obstacle_counts = np.zeros_like(primary_obstacle_counts)
        primary_ground_counts = np.bincount(
            ids[ground_valid], minlength=cells * cells
        )
        secondary_ground_counts = np.zeros_like(primary_ground_counts)
    else:
        primary_mask = source_labels[valid] == 0
        secondary_mask = source_labels[valid] == 1
        primary_obstacle_counts = np.bincount(
            ids[obstacle_valid & primary_mask], minlength=cells * cells
        )
        secondary_obstacle_counts = np.bincount(
            ids[obstacle_valid & secondary_mask], minlength=cells * cells
        )
        primary_ground_counts = np.bincount(
            ids[ground_valid & primary_mask], minlength=cells * cells
        )
        secondary_ground_counts = np.bincount(
            ids[ground_valid & secondary_mask], minlength=cells * cells
        )
    ground_counts = (
        primary_ground_counts + secondary_ground_counts
    ).reshape(cells, cells)
    primary_counts = primary_obstacle_counts.reshape(cells, cells)
    secondary_counts = secondary_obstacle_counts.reshape(cells, cells)

    # Preserve the raw scan texture. Nearby returns accumulate into brighter
    # pixels while single returns remain visible. Ground returns from both
    # sensors collapse into a neutral gray layer; obstacles keep their colors.
    ground_strength = np.clip(
        np.log1p(ground_counts) / math.log(12.0), 0.0, 1.0
    )
    primary_strength = np.clip(np.log1p(primary_counts) / math.log(8.0), 0.0, 1.0)
    secondary_strength = np.clip(
        np.log1p(secondary_counts) / math.log(8.0), 0.0, 1.0
    )
    raster = np.zeros((cells, cells, 3), dtype=np.float32)
    # Keep floor returns unmistakably neutral gray, including fused L1 returns.
    ground_level = 128 + 56 * ground_strength
    raster[..., 0] = ground_level * (ground_counts > 0)
    raster[..., 1] = ground_level * (ground_counts > 0)
    raster[..., 2] = ground_level * (ground_counts > 0)
    raster[..., 0] = (
        raster[..., 0]
        + 30 * (primary_counts > 0) + 85 * primary_strength
        + 28 * (secondary_counts > 0) + 45 * secondary_strength
    )
    raster[..., 1] = (
        raster[..., 1]
        + 145 * (primary_counts > 0) + 110 * primary_strength
        + 135 * (secondary_counts > 0) + 70 * secondary_strength
    )
    raster[..., 2] = (
        raster[..., 2]
        + 18 * (primary_counts > 0) + 35 * primary_strength
        + 170 * (secondary_counts > 0) + 75 * secondary_strength
    )
    raster = np.clip(raster, 0, 255).astype(np.uint8)

    plot = Image.fromarray(np.flipud(raster), mode="RGB")

    margin_left = 82
    margin_right = 250
    header_h = 142
    footer_h = 104
    width = margin_left + plot.width + margin_right
    height = header_h + plot.height + footer_h
    image = Image.new("RGB", (width, height), (0, 0, 0))
    image.paste(plot, (margin_left, header_h))
    draw = ImageDraw.Draw(image)

    title_font = load_font(29, bold=True)
    subtitle_font = load_font(17)
    label_font = load_font(16)
    small_font = load_font(14)

    fused = secondary_frame_count > 0
    title = (
        "Go2 IMU-Centric Dual-LiDAR Top-Down"
        if fused
        else "Go2 IMU-Centric MID-360 Top-Down"
    )
    draw.text((28, 20), title, font=title_font, fill=(235, 255, 225))
    draw.text(
        (28, 60),
        (
            "SCAN dual-cylinder clearance %.3f m | gray = ground | color = obstacle"
            % clearance["nearest_m"]
            if fused
            else "Raw LiDAR returns | black = no return | brighter green = denser returns"
        ),
        font=subtitle_font,
        fill=(135, 220, 105),
    )
    draw.text(
        (28, 87),
        (
            "Origin = Go2 IMU | r %.2f m | front/rear offset %.2f m | ceiling > +%.2f m removed"
            % (cylinder_radius, cylinder_offset, max_height)
        ),
        font=small_font,
        fill=(98, 155, 80),
    )
    draw.text(
        (28, 110),
        "Ground threshold: floor +%.2f m | clearance uses SCAN-Planner 5 cm occupied voxels"
        % OBSTACLE_MIN_HEIGHT_M,
        font=small_font,
        fill=(98, 155, 80),
    )

    plot_left = margin_left
    plot_top = header_h
    plot_right = margin_left + plot.width
    plot_bottom = header_h + plot.height

    def world_to_pixel(wx, wy):
        px = plot_left + (wx + range_m) / projection_resolution
        py = plot_bottom - (wy + range_m) / projection_resolution
        return px, py

    draw.rectangle((plot_left - 1, plot_top - 1, plot_right, plot_bottom), outline=(40, 62, 34))
    for metre in range(int(math.ceil(-range_m)), int(math.floor(range_m)) + 1):
        if metre == 0 or metre % 2 != 0:
            continue
        px, _ = world_to_pixel(metre, 0.0)
        draw.text((px - 8, plot_bottom + 8), "%+dm" % metre, font=small_font, fill=(98, 130, 86))

    heading_cosine = math.cos(float(map_heading_rad))
    heading_sine = math.sin(float(map_heading_rad))
    x0, y0 = world_to_pixel(0.0, 0.0)
    heading_end = world_to_pixel(heading_cosine, heading_sine)
    draw.line((x0, y0, heading_end[0], heading_end[1]), fill=(255, 205, 55), width=6)
    arrow_dx = heading_end[0] - x0
    arrow_dy = heading_end[1] - y0
    arrow_norm = max(math.hypot(arrow_dx, arrow_dy), 1e-6)
    arrow_ux = arrow_dx / arrow_norm
    arrow_uy = arrow_dy / arrow_norm
    arrow_px = -arrow_uy
    arrow_py = arrow_ux
    arrow_tip = (
        heading_end[0] + 18.0 * arrow_ux,
        heading_end[1] + 18.0 * arrow_uy,
    )
    draw.polygon(
        [
            arrow_tip,
            (
                heading_end[0] + 11.0 * arrow_px,
                heading_end[1] + 11.0 * arrow_py,
            ),
            (
                heading_end[0] - 11.0 * arrow_px,
                heading_end[1] - 11.0 * arrow_py,
            ),
        ],
        fill=(255, 205, 55),
    )

    footprint_radius_px = cylinder_radius / projection_resolution
    for ahead in (cylinder_offset, -cylinder_offset):
        center_x, center_y = world_to_pixel(
            ahead * heading_cosine,
            ahead * heading_sine,
        )
        draw.ellipse(
            (
                center_x - footprint_radius_px,
                center_y - footprint_radius_px,
                center_x + footprint_radius_px,
                center_y + footprint_radius_px,
            ),
            outline=(72, 158, 230),
            width=3,
        )
        draw.line(
            (center_x - 5, center_y, center_x + 5, center_y),
            fill=(120, 195, 255),
            width=2,
        )
        draw.line(
            (center_x, center_y - 5, center_x, center_y + 5),
            fill=(120, 195, 255),
            width=2,
        )

    legend_x = plot_right + 22
    legend_y = plot_top + 8
    draw.text((legend_x, legend_y), "GROUND / OBSTACLE", font=label_font, fill=(225, 245, 218))
    draw.rectangle(
        (legend_x, legend_y + 30, legend_x + 22, legend_y + 72),
        fill=(82, 87, 96),
        outline=(125, 131, 141),
    )
    draw.text(
        (legend_x + 32, legend_y + 41),
        "ground",
        font=small_font,
        fill=(185, 190, 198),
    )
    obstacle_legend_y = legend_y + 88
    draw.text(
        (legend_x, obstacle_legend_y),
        "obstacle density",
        font=small_font,
        fill=(190, 230, 175),
    )
    for index in range(100):
        value = index / 99.0
        colour = (
            int(30 + 85 * value),
            int(145 + 110 * value),
            int(18 + 35 * value),
        )
        draw.line(
            (
                legend_x,
                obstacle_legend_y + 25 + index,
                legend_x + 22,
                obstacle_legend_y + 25 + index,
            ),
            fill=colour,
        )
    draw.text(
        (legend_x + 32, obstacle_legend_y + 20),
        "sparse",
        font=small_font,
        fill=(190, 230, 175),
    )
    draw.text(
        (legend_x + 32, obstacle_legend_y + 115),
        "dense",
        font=small_font,
        fill=(190, 230, 175),
    )

    nav_y = obstacle_legend_y + 159
    draw.text(
        (legend_x, nav_y),
        "NAVIGATION",
        font=label_font,
        fill=(210, 239, 246),
    )
    draw.line(
        (legend_x, nav_y + 31, legend_x + 30, nav_y + 31),
        fill=(255, 205, 55),
        width=3,
    )
    draw.polygon(
        [
            (legend_x + 38, nav_y + 31),
            (legend_x + 30, nav_y + 26),
            (legend_x + 30, nav_y + 36),
        ],
        fill=(255, 205, 55),
    )
    draw.text(
        (legend_x + 46, nav_y + 22),
        "current heading",
        font=small_font,
        fill=(255, 225, 125),
    )
    draw.line(
        (legend_x, nav_y + 57, legend_x + 34, nav_y + 57),
        fill=(72, 194, 220),
        width=3,
    )
    draw.ellipse(
        (legend_x + 32, nav_y + 53, legend_x + 40, nav_y + 61),
        fill=(72, 194, 220),
    )
    draw.text(
        (legend_x + 46, nav_y + 48),
        "candidate route",
        font=small_font,
        fill=(150, 226, 238),
    )

    candidate_lines = [
        "%s %.2f m" % (item["direction"], item["distance_m"])
        for item in navigation["candidates"][:4]
    ]
    if not candidate_lines:
        candidate_lines = ["none >= 0.75 m"]
    nav_text_y = nav_y + 80
    draw.text(
        (legend_x, nav_text_y),
        (
            "world axes: fixed | yellow arrow = dog heading\n"
            "open sectors: %d\n"
            "forward %.2f m | left %.2f m | right %.2f m\n"
            "routes:\n  %s"
        )
        % (
            navigation["open_sectors"],
            navigation["forward_m"],
            navigation["left_m"],
            navigation["right_m"],
            "\n  ".join(candidate_lines),
        ),
        font=small_font,
        fill=(135, 199, 211),
        spacing=6,
    )

    info_y = nav_text_y + 126
    draw.text(
        (legend_x, info_y),
        (
            "%.0f m x %.0f m view\n%.1f cm display pixel\n%.0f cm map resolution\n"
            "CLEARANCE %.3f m\nfront %.3f m | rear %.3f m"
        )
        % (
            2.0 * range_m,
            2.0 * range_m,
            projection_resolution * 100.0,
            map_resolution * 100.0,
            clearance["nearest_m"],
            clearance["front_m"],
            clearance["rear_m"],
        ),
        font=small_font,
        fill=(150, 185, 137),
        spacing=7,
    )
    if frame_times is not None and len(frame_times) > 1:
        frame_span = max(frame_times[-1] - frame_times[0], 1e-6)
        rate = (len(frame_times) - 1) / frame_span
    else:
        rate = 0.0
    footer_y = plot_bottom + 31
    if frame_times is None:
        footer = "Static map | Captured: %s points | Shown after ceiling filter: %s points | Ground z %.3f m" % (
            format(int(points.shape[0]), ","),
            format(int(x.size), ","),
            ground_z,
        )
    else:
        source_text = (
            " | MID-360 frames: %d | Go2 frames: %d | Go2 points: %s"
            % (
                len(frame_times),
                secondary_frame_count,
                format(int(secondary_points_captured), ","),
            )
            if fused
            else ""
        )
        footer = (
            "Frames: %d | Captured: %s points | Shown after ceiling filter: %s points | %.2f Hz | Ground z %.3f m%s"
            % (
                len(frame_times),
                format(int(sum(point_counts)), ","),
                format(int(x.size), ","),
                rate,
                ground_z,
                source_text,
            )
        )
    draw.text((28, footer_y), footer, font=small_font, fill=(130, 165, 118))
    draw.text(
        (28, footer_y + 24),
        (
            "Dual-cylinder: radius %.2f m, offset %.2f m | "
            "MID-360/L1 transformed into Go2 IMU frame"
        )
        % (cylinder_radius, cylinder_offset),
        font=small_font,
        fill=(90, 145, 175),
    )

    metadata = PngInfo()
    metadata.add_text(
        "Description",
        "Go2 fused MID-360 and native LiDAR point-cloud top-down view, ceiling filtered",
    )
    metadata.add_text("Frame", "Go2 IMU frame")
    metadata.add_text("MaxHeightAboveFloorM", "%.3f" % max_height)
    metadata.add_text("GroundThresholdAboveFloorM", "%.3f" % OBSTACLE_MIN_HEIGHT_M)
    metadata.add_text("GroundZM", "%.6f" % ground_z)
    metadata.add_text("DualCylinderRadiusM", "%.6f" % cylinder_radius)
    metadata.add_text("DualCylinderOffsetM", "%.6f" % cylinder_offset)
    metadata.add_text("ClearanceM", "%.6f" % clearance["nearest_m"])
    metadata.add_text("Heading", "world axes fixed; arrow follows dog yaw")
    metadata.add_text("OpenSectors", str(navigation["open_sectors"]))
    metadata.add_text(
        "CandidateDirections",
        "; ".join(
            "%s=%.3fm" % (item["direction"], item["distance_m"])
            for item in navigation["candidates"]
        ),
    )
    metadata.add_text(
        "MID360PositionImuM",
        "%.6f, %.6f, %.6f" % tuple(geometry["mid360_position_imu"]),
    )
    metadata.add_text(
        "L1PositionImuM",
        "%.6f, %.6f, %.6f" % tuple(geometry["l1_position_imu"]),
    )
    metadata.add_text(
        "L1BaseCorrection",
        "x=%.6f y=%.6f yaw=%.6f deg"
        % (
            geometry["l1_base_correction_xyz"][0],
            geometry["l1_base_correction_xyz"][1],
            geometry["l1_base_correction_yaw_deg"],
        ),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, pnginfo=metadata)
    return {
        "output": str(output),
        "style": "green_points",
        "projection_resolution_m": projection_resolution,
        "range_m": range_m,
        "max_height_m": max_height,
        "frames": len(frame_times) if frame_times is not None else 0,
        "secondary_frames": secondary_frame_count,
        "secondary_points": int(secondary_points_captured),
        "frame_rate_hz": rate,
        "points_captured": int(sum(point_counts)) if point_counts is not None else int(points.shape[0]),
        "points_after_ceiling_filter": int(x.size),
        "ground_returns": int(np.count_nonzero(ground_mask)),
        "obstacle_returns": int(np.count_nonzero(~ground_mask)),
        "ground_z_m": ground_z,
        "frame": "Go2 IMU centre / world-fixed axes",
        "dual_cylinder_radius_m": cylinder_radius,
        "dual_cylinder_offset_m": cylinder_offset,
        "clearance_m": clearance["nearest_m"],
        "front_clearance_m": clearance["front_m"],
        "rear_clearance_m": clearance["rear_m"],
        "heading": "world axes fixed; arrow follows dog yaw",
        "map_origin_base_xy_m": (
            geometry["imu_position_base"][:2].astype(float).tolist()
        ),
        "open_sectors": navigation["open_sectors"],
        "candidate_directions": navigation["candidates"],
        "forward_travel_m": navigation["forward_m"],
        "left_travel_m": navigation["left_m"],
        "right_travel_m": navigation["right_m"],
        "mid360_position_imu_m": geometry["mid360_position_imu"].tolist(),
        "l1_position_imu_m": geometry["l1_position_imu"].tolist(),
        "l1_base_correction": {
            "x_m": round(float(geometry["l1_base_correction_xyz"][0]), 6),
            "y_m": round(float(geometry["l1_base_correction_xyz"][1]), 6),
            "yaw_deg": round(
                float(geometry["l1_base_correction_yaw_deg"]), 6
            ),
        },
        "map_panel_box": [
            int(margin_left),
            int(header_h),
            int(plot_right),
            int(plot_bottom),
        ],
    }


def render_topdown(
    points,
    frame_times,
    point_counts,
    resolution,
    range_m,
    max_height,
    pixels_per_cell,
    output,
):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    in_range = finite & (x >= -range_m) & (x < range_m) & (y >= -range_m) & (y < range_m)
    x, y, z = x[in_range], y[in_range], z[in_range]
    if x.size == 0:
        raise RuntimeError("No points fall inside the requested top-down range")

    ground_z = estimate_ground_z(np.column_stack((x, y, z)))
    relative_z = z - ground_z

    cells = max(1, int(round(2.0 * range_m / resolution)))

    ix = np.floor((x + range_m) / resolution).astype(np.int32)
    iy = np.floor((y + range_m) / resolution).astype(np.int32)
    valid = (ix >= 0) & (ix < cells) & (iy >= 0) & (iy < cells)
    ix, iy, relative_z = ix[valid], iy[valid], relative_z[valid]
    cell_id = iy * cells + ix

    floor_mask = relative_z < 0.12
    object_mask = (relative_z >= 0.12) & (relative_z <= max_height)
    floor_counts = np.bincount(cell_id[floor_mask], minlength=cells * cells)
    object_counts = np.bincount(cell_id[object_mask], minlength=cells * cells)
    object_height = np.zeros(cells * cells, dtype=np.float32)
    np.maximum.at(object_height, cell_id[object_mask], relative_z[object_mask])

    floor_density = np.log1p(floor_counts.reshape(cells, cells)) / math.log(32.0)
    floor_density = gaussian_blur_2d(np.clip(floor_density, 0.0, 1.0), 0.5)

    object_raw = object_counts.reshape(cells, cells).astype(np.float32)
    object_fraction = np.log1p(object_raw) / math.log(10.0)
    object_fraction = np.clip(object_fraction, 0.0, 1.0)
    height_grid = object_height.reshape(cells, cells)
    weight = object_raw > 0

    blurred_weight = gaussian_blur_2d(weight.astype(np.float32), 0.5)
    blurred_height = gaussian_blur_2d(height_grid * weight, 0.5)
    soft_height = np.divide(
        blurred_height,
        blurred_weight,
        out=np.zeros_like(blurred_height),
        where=blurred_weight > 1e-5,
    )
    soft_occupancy = gaussian_blur_2d(object_fraction, 0.45)
    height_fraction = np.clip(soft_height / max(max_height, 1e-5), 0.0, 1.0)

    grad_y, grad_x = np.gradient(height_fraction)
    normal_x = -6.0 * grad_x
    normal_y = -6.0 * grad_y
    normal_z = np.ones_like(normal_x)
    normal_length = np.sqrt(normal_x ** 2 + normal_y ** 2 + normal_z ** 2)
    light_x, light_y, light_z = -0.45, -0.65, 0.62
    light_length = math.sqrt(light_x ** 2 + light_y ** 2 + light_z ** 2)
    illumination = (
        normal_x * light_x + normal_y * light_y + normal_z * light_z
    ) / (normal_length * light_length)
    relief = 0.62 + 0.58 * np.clip(illumination, 0.0, 1.0)

    background = np.asarray([7, 9, 13], dtype=np.float32)
    floor_colour = np.asarray([34, 40, 49], dtype=np.float32)
    floor_alpha = np.clip(floor_density * 0.72, 0.0, 0.72)
    raster = background + (floor_colour - background) * floor_alpha[..., None]

    obstacle_colour = height_colours(height_fraction) * relief[..., None]
    obstacle_alpha = np.clip(soft_occupancy * 1.18, 0.0, 0.92)
    raster = raster * (1.0 - obstacle_alpha[..., None]) + obstacle_colour * obstacle_alpha[..., None]
    raster = np.clip(raster, 0.0, 255.0).astype(np.uint8)
    raster = np.flipud(raster)

    scale = max(1, int(pixels_per_cell))
    plot = Image.fromarray(raster, mode="RGB").resize(
        (cells * scale, cells * scale), resample=Image.BICUBIC
    )

    margin_left = 95
    margin_right = 290
    header_h = 132
    footer_h = 128
    width = margin_left + plot.width + margin_right
    height = header_h + plot.height + footer_h
    image = Image.new("RGB", (width, height), (15, 18, 24))
    image.paste(plot, (margin_left, header_h))
    draw = ImageDraw.Draw(image)

    title_font = load_font(30, bold=True)
    subtitle_font = load_font(18)
    label_font = load_font(17)
    small_font = load_font(15)
    legend_font = load_font(17)

    draw.text(
        (32, 24),
        "MID-360 Point Cloud - Top-Down View",
        font=title_font,
        fill=(244, 247, 250),
    )
    draw.text(
        (32, 66),
        "5 cm data grid | soft height and return-density shading | +X forward, +Y left",
        font=subtitle_font,
        fill=(176, 185, 198),
    )
    draw.text(
        (32, 95),
        "Source: Go2 MID-360 /livox/lidar | LiDAR frame | floor-relative height",
        font=small_font,
        fill=(132, 145, 162),
    )

    plot_left = margin_left
    plot_top = header_h
    plot_right = margin_left + plot.width
    plot_bottom = header_h + plot.height
    axis_color = (82, 91, 105)
    grid_color = (25, 30, 38)

    def world_to_pixel(wx, wy):
        px = plot_left + (wx + range_m) / resolution * scale
        py = plot_bottom - (wy + range_m) / resolution * scale
        return px, py

    first_grid = int(math.ceil(-range_m))
    last_grid = int(math.floor(range_m))
    for metre in range(first_grid, last_grid + 1):
        if metre == 0:
            continue
        px, _ = world_to_pixel(metre, 0.0)
        _, py = world_to_pixel(0.0, metre)
        draw.line((px, plot_top, px, plot_bottom), fill=grid_color, width=1)
        draw.line((plot_left, py, plot_right, py), fill=grid_color, width=1)
        if metre % 2 == 0:
            draw.text((px - 9, plot_bottom + 7), "%+dm" % metre, font=small_font, fill=(145, 156, 172))
            draw.text((plot_left - 52, py - 9), "%+dm" % metre, font=small_font, fill=(145, 156, 172))

    x0, y0 = world_to_pixel(0.0, 0.0)
    draw.line((plot_left, y0, plot_right, y0), fill=axis_color, width=2)
    draw.line((x0, plot_top, x0, plot_bottom), fill=axis_color, width=2)

    axis_length = 1.0 / resolution * scale
    draw.line((x0, y0, x0 + axis_length, y0), fill=(255, 92, 92), width=5)
    draw.polygon(
        [
            (x0 + axis_length + 14, y0),
            (x0 + axis_length - 2, y0 - 8),
            (x0 + axis_length - 2, y0 + 8),
        ],
        fill=(255, 92, 92),
    )
    draw.line((x0, y0, x0, y0 - axis_length), fill=(92, 225, 132), width=5)
    draw.polygon(
        [
            (x0, y0 - axis_length - 14),
            (x0 - 8, y0 - axis_length + 2),
            (x0 + 8, y0 - axis_length + 2),
        ],
        fill=(92, 225, 132),
    )
    draw.text((x0 + axis_length + 20, y0 - 12), "+X", font=label_font, fill=(255, 120, 120))
    draw.text((x0 + 10, y0 - axis_length - 27), "+Y", font=label_font, fill=(120, 235, 155))

    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    overlay_draw = ImageDraw.Draw(overlay)
    body_left, body_bottom = world_to_pixel(-0.38, -0.19)
    body_right, body_top = world_to_pixel(0.38, 0.19)
    overlay_draw.rectangle(
        (body_left, body_top, body_right, body_bottom),
        fill=(255, 220, 70, 55),
        outline=(255, 226, 94, 255),
        width=4,
    )
    overlay_draw.line((x0, y0, x0 + axis_length * 1.25, y0), fill=(255, 226, 94, 255), width=4)
    overlay_draw.text((x0 + 18, y0 + 12), "ROBOT / FRONT", font=small_font, fill=(255, 235, 150, 255))
    image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
    draw = ImageDraw.Draw(image)

    scale_bar_m = 1.0
    scale_bar_px = int(scale_bar_m / resolution * scale)
    bar_x = plot_left + 24
    bar_y = plot_bottom - 28
    draw.rectangle((bar_x - 10, bar_y - 35, bar_x + scale_bar_px + 10, bar_y + 13), fill=(15, 18, 24))
    draw.line((bar_x, bar_y, bar_x + scale_bar_px, bar_y), fill=(235, 240, 245), width=6)
    draw.line((bar_x, bar_y - 8, bar_x, bar_y + 8), fill=(235, 240, 245), width=3)
    draw.line(
        (bar_x + scale_bar_px, bar_y - 8, bar_x + scale_bar_px, bar_y + 8),
        fill=(235, 240, 245),
        width=3,
    )
    draw.text((bar_x, bar_y - 30), "1 metre", font=small_font, fill=(235, 240, 245))

    legend_x = plot_right + 24
    legend_y = plot_top + 8
    draw.text((legend_x, legend_y), "HEIGHT / DENSITY", font=label_font, fill=(232, 237, 243))
    legend_y += 40
    gradient_height = 190
    gradient_width = 28
    for offset in range(gradient_height):
        fraction = 1.0 - offset / float(gradient_height - 1)
        colour = tuple(int(value) for value in height_colours(np.asarray([fraction]))[0])
        draw.line(
            (legend_x, legend_y + offset, legend_x + gradient_width, legend_y + offset),
            fill=colour,
        )
    draw.rectangle(
        (
            legend_x,
            legend_y,
            legend_x + gradient_width,
            legend_y + gradient_height,
        ),
        outline=(105, 115, 130),
    )
    draw.text((legend_x - 4, legend_y - 23), "%.1f m" % max_height, font=small_font, fill=(210, 218, 228))
    draw.text((legend_x - 4, legend_y + gradient_height + 5), "0.12 m", font=small_font, fill=(210, 218, 228))

    legend_text_x = legend_x + 42
    draw.text((legend_text_x, legend_y + 5), "High obstacle", font=legend_font, fill=(190, 199, 211))
    draw.text((legend_text_x, legend_y + 31), "Medium", font=legend_font, fill=(190, 199, 211))
    draw.text((legend_text_x, legend_y + 58), "Low obstacle", font=legend_font, fill=(190, 199, 211))
    draw.text((legend_text_x, legend_y + 102), "Soft edge =", font=legend_font, fill=(190, 199, 211))
    draw.text((legend_text_x, legend_y + 126), "sparse returns", font=legend_font, fill=(190, 199, 211))

    legend_y += gradient_height + 58
    draw.rectangle((legend_x, legend_y, legend_x + 26, legend_y + 18), fill=(34, 40, 49), outline=(95, 105, 120))
    draw.text((legend_x + 38, legend_y - 2), "Floor / ground", font=legend_font, fill=(190, 199, 211))

    legend_y += 48
    draw.text(
        (legend_x, legend_y),
        "Data cells: %.2f m\nDisplay is smoothed\nfor depth perception." % resolution,
        font=legend_font,
        fill=(190, 199, 211),
        spacing=5,
    )

    if len(frame_times) > 1:
        frame_span = max(frame_times[-1] - frame_times[0], 1e-6)
        rate = (len(frame_times) - 1) / frame_span
    else:
        rate = 0.0
    footer_y = plot_bottom + 20
    draw.text(
        (
            32,
            footer_y,
        ),
        "Frames: %d | Points captured: %s | Points shown: %s | %.2f Hz | Ground z %.3f m"
        % (
            len(frame_times),
            format(int(sum(point_counts)), ","),
            format(int(x.size), ","),
            rate,
            ground_z,
        ),
        font=small_font,
        fill=(160, 171, 186),
    )
    draw.text(
        (32, footer_y + 32),
        "View window: %.1f m x %.1f m around LiDAR | height band: floor+0.12 m to floor+%.1f m"
        % (2.0 * range_m, 2.0 * range_m, max_height),
        font=small_font,
        fill=(160, 171, 186),
    )

    metadata = PngInfo()
    metadata.add_text("Description", "Go2 MID-360 five-second top-down occupancy view")
    metadata.add_text("Resolution", "%.3f metre per cell" % resolution)
    metadata.add_text("Frame", "livox_frame / LiDAR frame")
    metadata.add_text("Duration", "%.3f seconds" % (frame_times[-1] - frame_times[0] if len(frame_times) > 1 else 0.0))
    metadata.add_text("GroundZ", "%.6f" % ground_z)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, pnginfo=metadata)

    stats = {
        "output": str(output),
        "resolution_m": resolution,
        "range_m": range_m,
        "max_height_m": max_height,
        "frames": len(frame_times),
        "frame_rate_hz": rate,
        "points_captured": int(sum(point_counts)),
        "points_in_view": int(x.size),
        "ground_z_m": ground_z,
    }
    return stats


def main():
    args = parse_args()
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.resolution <= 0 or args.range_m <= 0:
        raise SystemExit("--resolution and --range must be positive")
    if args.max_height <= 0:
        raise SystemExit("--max-height must be positive")
    cylinder_radius, cylinder_offset, map_resolution = load_dual_cylinder_model(
        args.planner_config
    )
    geometry = sensor_geometry()
    if args.rolling:
        run_rolling_topdown(
            args,
            cylinder_radius,
            cylinder_offset,
            map_resolution,
            geometry,
        )
        return

    if args.input_pcd is not None:
        points = load_pcd_xyz(args.input_pcd)
        frame_times = None
        point_counts = None
        secondary_points = np.empty((0, 3))
        secondary_frame_times = []
        secondary_point_counts = []
        robot_pose = None
    else:
        secondary_topic = None if args.no_secondary else args.secondary_topic
        (
            points,
            frame_times,
            point_counts,
            secondary_points,
            secondary_frame_times,
            secondary_point_counts,
            robot_pose,
        ) = collect_cloud(
            args.topic,
            args.duration,
            args.timeout,
            secondary_topic,
            args.odom_topic,
        )
        # The MID-360 data is published in its LiDAR frame. Express both
        # LiDARs in the unchanged Go2 IMU frame before fusion or rendering.
        points = transform_xyz(points, geometry["lidar_to_imu"])
        if secondary_points.size:
            secondary_points = transform_xyz(
                secondary_points,
                geometry["base_to_imu"] @ geometry["l1_to_base"],
            )

    source_labels = None
    if secondary_points.size:
        points = np.concatenate((points[:, :3], secondary_points), axis=0)
        source_labels = np.concatenate(
            (
                np.zeros(len(points) - len(secondary_points), dtype=np.uint8),
                np.ones(len(secondary_points), dtype=np.uint8),
            )
        )
    if args.style == "green_points":
        stats = render_green_points_topdown(
            points,
            frame_times,
            point_counts,
            args.range_m,
            args.max_height,
            args.projection_resolution,
            args.output,
            geometry,
            cylinder_radius,
            cylinder_offset,
            map_resolution,
            source_labels=source_labels,
            secondary_frame_count=len(secondary_frame_times),
            secondary_points_captured=len(secondary_points),
        )
    else:
        stats = render_topdown(
            points,
            frame_times,
            point_counts,
            args.resolution,
            args.range_m,
            args.max_height,
            args.pixels_per_cell,
            args.output,
        )
    stats["robot_pose"] = robot_pose
    stats["robot_pose_available"] = robot_pose is not None
    stats["camera_url"] = args.camera_url
    stats["snapshot_sequence"] = args.snapshot_sequence

    if not args.no_raw and frame_times is not None:
        raw_path = args.output.with_suffix(".npz")
        np.savez_compressed(
            raw_path,
            points=points,
            frame_times=np.asarray(frame_times, dtype=np.float64),
            point_counts=np.asarray(point_counts, dtype=np.int64),
            secondary_points=secondary_points,
            secondary_frame_times=np.asarray(secondary_frame_times, dtype=np.float64),
            secondary_point_counts=np.asarray(secondary_point_counts, dtype=np.int64),
            resolution=args.resolution,
            range_m=args.range_m,
            frame="Go2 IMU",
            robot_pose_json=json.dumps(robot_pose),
            mid360_position_imu_m=geometry["mid360_position_imu"].tolist(),
            l1_position_imu_m=geometry["l1_position_imu"].tolist(),
        )
        stats["raw_data"] = str(raw_path)

    stats_path = args.output.with_suffix(".json")
    stats_path.write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()

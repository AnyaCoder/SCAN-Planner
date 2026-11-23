#!/usr/bin/env python3
"""Ask a VLM for one floor pixel and visualize its metric ground projection."""

import argparse
import json
import math
import os
import re
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from vlm_common import (
    QueryCancelled,
    QueryCancellation,
    StreamValueParser,
    load_dotenv,
    query_json,
)


LOCAL_WS = Path(__file__).resolve().parents[4]
CAMERA_ROUTER_SYSTEM_PROMPT = """The image is the dog's front fisheye camera.

If the next walkable floor point is clear, choose it near the centre of the
visible passage, about 0.6 to 4 m away. Do not choose furniture, walls, people, or
the horizon.

If the dog is very close to a wall, glass, door, furniture, or the horizon and
no floor is visible, return topdown. Do not put a point on a wall or reflection.

Use image coordinates: x=0 is left, x=1 is right, y=0 is top, and y=1 is bottom.
Return one line only:
point: x y
or
topdown"""
CAMERA_USER_PROMPT = "Choose the next walkable floor point for the dog."

TOPDOWN_LOCALIZATION_SYSTEM_PROMPT = """The first image is the dog's front
camera. It is blocked by the nearby wall or no floor is visible.

The second image is the 10 m by 10 m top-down map. The dog is at the centre.
Choose one light-gray floor pixel in a continuous walkable passage, about
1.5 to 4 m from the dog. Choose near the centre of the long gray corridor,
not at a wall edge. Follow the open passage direction. The orange arrow is the
dog heading; if it points into a green wall, do not choose that way. Do not
choose green, cyan, black, white text, the robot, or an obstacle.

Use top-down image coordinates: x=0 is left, x=1 is right, y=0 is top, and
y=1 is bottom. Return one line only:
point: x y"""

_NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
VISIBLE_DECISION_RE = re.compile(
    r"^\s*(?:"
    r"(?:(?:point|map_point)\s*[:=]\s*"
    + _NUMBER
    + r"\s+"
    + _NUMBER
    + r")"
    r"|(?:(?:view|context)\s*[:=]\s*)?(?:topdown|map)"
    r")\s*[.!]?\s*$",
    re.IGNORECASE,
)


class DecisionStreamFilter:
    """Expose only a complete normalized point line from model content."""

    def __init__(self):
        self.pending = ""

    def feed(self, text, final=False):
        self.pending += text
        lines = self.pending.splitlines(keepends=True)
        if not final and lines and not lines[-1].endswith(("\n", "\r")):
            self.pending = lines.pop()
        else:
            self.pending = ""
        visible = [
            line for line in lines if VISIBLE_DECISION_RE.match(line)
        ]
        if final and self.pending:
            if VISIBLE_DECISION_RE.match(self.pending):
                visible.append(self.pending)
            self.pending = ""
        return "".join(visible)


def image_message_to_bgr(message):
    if message.encoding not in ("bgr8", "rgb8"):
        raise ValueError("unsupported image encoding %s" % message.encoding)
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = row[:, : message.width * 3].reshape(
        message.height, message.width, 3
    )
    if message.encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def bgr_to_image_message(image, header):
    message = Image()
    message.header = header
    message.height = int(image.shape[0])
    message.width = int(image.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = message.width * 3
    message.data = np.ascontiguousarray(image).tobytes()
    return message


def quaternion_matrix(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


def annotate_query_image(image):
    """Draw the normalized coordinate grid used for camera floor selection."""
    annotated = image.copy()
    height, width = annotated.shape[:2]
    for index in range(1, 10):
        x = int(width * index / 10.0)
        y = int(height * index / 10.0)
        cv2.line(annotated, (x, 0), (x, height), (205, 205, 205), 1)
        cv2.line(annotated, (0, y), (width, y), (205, 205, 205), 1)
    for index in range(10):
        x = int((width - 1) * index / 10.0)
        y = int((height - 1) * index / 10.0)
        cv2.putText(
            annotated,
            "%.1f" % (index / 10.0),
            (max(2, min(width - 42, x + 4)), 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (40, 230, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            "%.1f" % (index / 10.0),
            (4, min(height - 4, y + 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (40, 230, 255),
            1,
            cv2.LINE_AA,
        )
    return annotated


def neutral_floor_mask(image):
    """Match the neutral-gray floor color used by the top-down renderer."""
    pixels = image.astype(np.int16)
    spread = pixels.max(axis=2) - pixels.min(axis=2)
    brightness = pixels.mean(axis=2)
    return (
        (spread <= 6)
        & (brightness >= 110.0)
        & (brightness <= 220.0)
    )


def annotate_topdown_query(image, range_m=5.0):
    """Emphasize gray floor returns and add normalized coordinates."""
    annotated = image.copy()
    floor = neutral_floor_mask(annotated)
    width = image.shape[1]
    center = width // 2
    keep_clear = max(4, int(round(1.50 * width / (2.0 * range_m))))
    cv2.circle(annotated, (center, center), keep_clear, (45, 45, 130), 3)
    cv2.circle(floor.view(np.uint8), (center, center), keep_clear, 0, -1)
    annotated[floor] = [190, 190, 190]
    height, width = annotated.shape[:2]
    for index in range(1, 10):
        x = int(round(index * (width - 1) / 10.0))
        y = int(round(index * (height - 1) / 10.0))
        cv2.line(annotated, (x, 0), (x, height - 1), (70, 70, 70), 1)
        cv2.line(annotated, (0, y), (width - 1, y), (70, 70, 70), 1)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(
        annotated, "x=0", (5, 18), font, 0.45, (230, 230, 230), 1, cv2.LINE_AA
    )
    cv2.putText(
        annotated,
        "x=1",
        (width - 42, 18),
        font,
        0.45,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        annotated,
        "y=0",
        (5, 36),
        font,
        0.45,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        annotated,
        "y=1",
        (5, height - 8),
        font,
        0.45,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )
    return annotated


def angle_difference(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def world_xyz_to_topdown_pixel(world_xyz, image_size, metadata):
    """Project one world point into the current on-demand map crop."""
    if world_xyz is None or len(world_xyz) < 2:
        return None
    map_range = float(metadata.get("range_m", 0.0))
    if map_range <= 0.0:
        return None
    if metadata.get("map_orientation", "current_heading") == "world_fixed":
        origin = metadata.get("map_origin_world_xyz_m")
        if origin is None:
            return None
        x_norm = 0.5 + (
            float(world_xyz[0]) - float(origin[0])
        ) / (2.0 * map_range)
        y_norm = 0.5 - (
            float(world_xyz[1]) - float(origin[1])
        ) / (2.0 * map_range)
    else:
        pose = metadata.get("robot_pose") or {}
        if not all(key in pose for key in ("x", "y", "yaw_rad")):
            return None
        delta_x = float(world_xyz[0]) - float(pose["x"])
        delta_y = float(world_xyz[1]) - float(pose["y"])
        cosine = math.cos(float(pose["yaw_rad"]))
        sine = math.sin(float(pose["yaw_rad"]))
        base_x = cosine * delta_x + sine * delta_y
        base_y = -sine * delta_x + cosine * delta_y
        origin = metadata.get("map_origin_base_xy_m", [0.0, 0.0])
        local_x = base_x - float(origin[0])
        local_y = base_y - float(origin[1])
        x_norm = 0.5 + local_x / (2.0 * map_range)
        y_norm = 0.5 - local_y / (2.0 * map_range)
    if not (0.0 <= x_norm <= 1.0 and 0.0 <= y_norm <= 1.0):
        return None
    width, height = int(image_size[0]), int(image_size[1])
    return (
        int(round(x_norm * (width - 1))),
        int(round(y_norm * (height - 1))),
    )


def topdown_pixel_to_world(x_norm, y_norm, metadata):
    """Convert one normalized top-down pixel to a world-floor point."""
    map_range = float(metadata.get("range_m", 0.0))
    if map_range <= 0.0:
        raise RuntimeError("top-down map has no metric range")
    if not (0.0 <= float(x_norm) <= 1.0 and 0.0 <= float(y_norm) <= 1.0):
        raise RuntimeError("VLM top-down point is outside the map")

    if metadata.get("map_orientation", "current_heading") == "world_fixed":
        origin = metadata.get("map_origin_world_xyz_m")
        if origin is None:
            raise RuntimeError("top-down map has no world origin")
        world_x = float(origin[0]) + (float(x_norm) - 0.5) * 2.0 * map_range
        world_y = float(origin[1]) - (float(y_norm) - 0.5) * 2.0 * map_range
    else:
        pose = metadata.get("robot_pose") or {}
        if not all(key in pose for key in ("x", "y", "yaw_rad")):
            raise RuntimeError("top-down map has no robot pose")
        origin = metadata.get("map_origin_base_xy_m", [0.0, 0.0])
        local_x = (
            float(origin[0])
            + (float(x_norm) - 0.5) * 2.0 * map_range
        )
        local_y = (
            float(origin[1])
            - (float(y_norm) - 0.5) * 2.0 * map_range
        )
        cosine = math.cos(float(pose["yaw_rad"]))
        sine = math.sin(float(pose["yaw_rad"]))
        world_x = (
            float(pose["x"]) + cosine * local_x - sine * local_y
        )
        world_y = (
            float(pose["y"]) + sine * local_x + cosine * local_y
        )

    robot_pose = metadata.get("robot_pose") or {}
    ground_z = metadata.get("ground_z_m")
    if ground_z is None or "z" not in robot_pose:
        raise RuntimeError("top-down map has no floor height")
    return np.asarray(
        [world_x, world_y, float(robot_pose["z"]) + float(ground_z)],
        dtype=np.float64,
    )


def snap_to_nearest_gray_floor(image, x_norm, y_norm, metadata):
    """Move a close VLM pixel onto gray floor while preserving map scale."""
    if image is None or image.size == 0:
        return None
    height, width = image.shape[:2]
    u = int(round(float(x_norm) * (width - 1)))
    v = int(round(float(y_norm) * (height - 1)))
    map_range = max(0.1, float(metadata.get("range_m", 5.0)))
    radius = max(4, int(round(0.21 * width / (2.0 * map_range))))
    left, right = max(0, u - radius), min(width, u + radius + 1)
    top, bottom = max(0, v - radius), min(height, v + radius + 1)
    patch = image[top:bottom, left:right]
    rows, columns = np.nonzero(
        neutral_floor_mask(patch)
    )
    if rows.size == 0:
        return None
    distances = np.hypot(
        rows + top - v,
        columns + left - u,
    )
    index = int(np.argmin(distances))
    snapped_u = int(columns[index] + left)
    snapped_v = int(rows[index] + top)
    return {
        "x": round(snapped_u / max(1, width - 1), 6),
        "y": round(snapped_v / max(1, height - 1), 6),
        "offset_m": round(
            float(distances[index]) * 2.0 * map_range / max(1, width), 4
        ),
    }


def overlay_decision_history(image, metadata, world_points):
    """Draw selected VLM world points and their decision-order trajectory."""
    annotated = image.copy()
    pixels = []
    for world_point in world_points:
        pixel = world_xyz_to_topdown_pixel(
            world_point, (image.shape[1], image.shape[0]), metadata
        )
        if pixel is not None:
            pixels.append(pixel)
    if len(pixels) >= 2:
        cv2.polylines(
            annotated,
            [np.asarray(pixels, dtype=np.int32)],
            False,
            (220, 188, 82),
            2,
            cv2.LINE_AA,
        )
    for pixel in pixels:
        cv2.circle(
            annotated,
            pixel,
            4,
            (0, 145, 255),
            -1,
            cv2.LINE_AA,
        )
    return annotated


def normalize_turn_delta_deg(action, value):
    angle = float(value)
    if action == "turn_left":
        angle = abs(angle) if angle != 0.0 else 45.0
        return max(20.0, min(90.0, angle))
    if action == "turn_right":
        angle = -abs(angle) if angle != 0.0 else -45.0
        return min(-20.0, max(-90.0, angle))
    if action == "turn_around":
        return -180.0 if angle < 0.0 else 180.0
    return 0.0


def action_from_bearing(bearing_deg):
    """Derive compatibility metadata from the selected point direction."""
    absolute = abs(float(bearing_deg))
    if absolute <= 45.0:
        return "forward"
    if absolute >= 135.0:
        return "turn_around"
    return "turn_left" if bearing_deg > 0.0 else "turn_right"


def target_bearing_deg(metadata, origin, target):
    """Target bearing relative to robot heading; positive is left."""
    yaw = float((metadata.get("robot_pose") or {}).get("yaw_rad", 0.0))
    world_bearing = math.atan2(
        float(target[1]) - float(origin[1]),
        float(target[0]) - float(origin[0]),
    )
    return math.degrees(angle_difference(world_bearing, yaw))


class VlmPixelShadow(Node):
    def __init__(self, args):
        super().__init__("vlm_pixel_shadow")
        self.args = args
        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_image_header = None
        self.latest_camera_info = None
        self.latest_cloud = None
        self.latest_cloud_header = None
        self.busy = False
        self.last_query_at = 0.0
        self.locked_result = None
        self.goal_published = False
        self.query_generation = 0
        self.active_cancel_event = None
        self.chat_history = []
        self.decision_history_world = []
        self.next_request_reason = "initial navigation request"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        camera_info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Image, args.image_topic, self.image_callback, image_qos
        )
        self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self.camera_info_callback,
            camera_info_qos,
        )
        self.create_subscription(
            PointCloud2,
            args.ground_cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, args.marker_topic, marker_qos
        )
        self.image_pub = self.create_publisher(
            Image, args.image_output_topic, image_qos
        )
        self.status_pub = self.create_publisher(
            String, args.status_topic, status_qos
        )
        stream_qos = QoSProfile(
            depth=500,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.stream_pub = self.create_publisher(
            String, args.stream_topic, stream_qos
        )
        topdown_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.topdown_pub = self.create_publisher(
            Image, args.topdown_topic, topdown_qos
        )
        self.last_topdown_mtime_ns = None
        self.goal_pub = None
        self.approval_sub = None
        if args.drive:
            self.goal_pub = self.create_publisher(
                PoseStamped, args.goal_topic, 10
            )
            self.approval_sub = self.create_subscription(
                String,
                args.approval_topic,
                self.approval_callback,
                10,
            )
        self.create_timer(0.5, self.maybe_query)
        self.create_timer(0.5, self.publish_topdown_if_updated)
        self.get_logger().info(
            "VLM pixel shadow ready; one-shot drive=%s"
            % args.drive
        )

    def image_callback(self, message):
        try:
            image = image_message_to_bgr(message)
        except Exception as exc:
            self.get_logger().warning("image rejected: %s" % exc)
            return
        with self.lock:
            self.latest_image = image
            self.latest_image_header = message.header
            locked_result = self.locked_result
            camera_info = self.latest_camera_info
        if (
            locked_result is not None
            and camera_info is not None
            and locked_result.get("target_xyz")
        ):
            try:
                locked_result = self.reproject_locked_result(
                    locked_result, message.header, camera_info
                )
            except (
                TransformException,
                ValueError,
                cv2.error,
            ):
                locked_result = dict(locked_result)
                locked_result["projection_stale"] = True
                locked_result["camera_pixel"] = None
                locked_result["visible"] = False
            self.publish_result(
                locked_result, image, message.header, republish=True
            )
        else:
            self.image_pub.publish(bgr_to_image_message(image, message.header))

    def camera_info_callback(self, message):
        with self.lock:
            self.latest_camera_info = message

    def cloud_callback(self, message):
        with self.lock:
            self.latest_cloud = message
            self.latest_cloud_header = message.header

    def decision_history_snapshot(self):
        with self.lock:
            return [list(point) for point in self.decision_history_world]

    def chat_history_snapshot(self):
        with self.lock:
            return [dict(message) for message in self.chat_history]

    def current_navigation_metadata(self):
        path = Path(self.args.navigation_metadata_path)
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}

    def decorate_topdown_history(self, image):
        metadata = self.current_navigation_metadata()
        return overlay_decision_history(
            image, metadata, self.decision_history_snapshot()
        )

    def record_approved_decision(
        self, result, request_reason, navigation_metadata
    ):
        if (
            not result.get("walkable")
            or not result.get("target_xyz")
        ):
            return
        world_point = [
            round(float(value), 4) for value in result["target_xyz"]
        ]
        snapshot = navigation_metadata.get("snapshot") or {}
        decision_time = (
            snapshot.get("captured_at")
            or datetime.now().astimezone().isoformat(timespec="seconds")
        )
        assistant_content = json.dumps(
            {
                "time": decision_time,
                "world_xyz": world_point,
            },
            separators=(",", ":"),
        )
        user_content = (
            "Choose the next walkable point. Navigation request: %s."
            % request_reason
        )
        with self.lock:
            self.chat_history.extend(
                [
                    {"role": "user", "content": user_content},
                    {"role": "assistant", "content": assistant_content},
                ]
            )
            # Only approved decisions enter action memory, and only the newest
            # entries are sent back to the model.
            self.chat_history = self.chat_history[-16:]
            self.decision_history_world.append(world_point)
            self.decision_history_world = self.decision_history_world[-200:]
            self.next_request_reason = "next navigation decision"
        result["world_decision"] = {
            "time": decision_time,
            "world_xyz": world_point,
        }
        result["chat_history_messages"] = len(self.chat_history)

    def publish_topdown_if_updated(self):
        path = Path(self.args.topdown_image_path)
        try:
            mtime_ns = path.stat().st_mtime_ns
        except OSError:
            return
        if mtime_ns == self.last_topdown_mtime_ns:
            return
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            return
        with self.lock:
            recent_result = (
                dict(self.locked_result)
                if self.locked_result is not None
                else None
            )
        image = self.publish_topdown_result(
            recent_result, image=image, publish=False
        )
        self.publish_topdown_image(image)
        self.last_topdown_mtime_ns = mtime_ns

    def publish_topdown_image(self, image):
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.height = int(image.shape[0])
        message.width = int(image.shape[1])
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = np.ascontiguousarray(image).tobytes()
        self.topdown_pub.publish(message)

    def publish_topdown_result(self, result, image=None, publish=True):
        if image is None:
            image = cv2.imread(
                str(self.args.topdown_image_path), cv2.IMREAD_COLOR
            )
        if image is None:
            return None
        image = self.decorate_topdown_history(image)
        pixel = (result or {}).get("map_pixel")
        if (result or {}).get("walkable") and pixel:
            u, v = int(pixel[0]), int(pixel[1])
            cv2.circle(image, (u, v), 20, (0, 80, 255), 4, cv2.LINE_AA)
            cv2.line(image, (u - 30, v), (u - 12, v), (0, 80, 255), 3)
            cv2.line(image, (u + 12, v), (u + 30, v), (0, 80, 255), 3)
            cv2.line(image, (u, v - 30), (u, v - 12), (0, 80, 255), 3)
            cv2.line(image, (u, v + 12), (u, v + 30), (0, 80, 255), 3)
        if publish:
            self.publish_topdown_image(image)
        return image

    def publish_stream_event(self, event):
        self.stream_pub.publish(
            String(data=json.dumps(event, ensure_ascii=False))
        )

    def maybe_query(self):
        with self.lock:
            if self.args.lock_target and self.locked_result is not None:
                return
        now = time.monotonic()
        if now - self.last_query_at < self.args.interval:
            return
        with self.lock:
            if (
                self.busy
                or self.latest_image is None
                or self.latest_camera_info is None
                or self.latest_cloud is None
            ):
                return
            query_generation = self.query_generation
            request_reason = self.next_request_reason
            cancel_event = QueryCancellation()
            self.active_cancel_event = cancel_event
            self.busy = True
            self.last_query_at = now
        threading.Thread(
            target=self.query_worker,
            args=(
                query_generation,
                request_reason,
                cancel_event,
            ),
            name="vlm-pixel-query",
            daemon=True,
        ).start()

    def query_worker(
        self,
        query_generation,
        request_reason,
        cancel_event,
    ):
        try:
            # Every VLM request is an on-demand observation. The five-second
            # point-cloud window is already in Go2 memory, so this renders a
            # snapshot without starting another scan.
            self.get_logger().info(
                "Capturing a fresh navigation snapshot for query generation %d"
                % query_generation
            )
            (
                navigation_camera_path,
                navigation_topdown_path,
                navigation_metadata,
            ) = self.prepare_navigation_context()
            navigation_prompt_path = Path(
                self.args.navigation_prompt_path
            )
            if cancel_event.is_set():
                raise QueryCancelled("VLM generation cancelled during snapshot")
            planner_context = {
                "forward_travel_m": navigation_metadata.get(
                    "forward_travel_m"
                ),
                "left_travel_m": navigation_metadata.get("left_travel_m"),
                "right_travel_m": navigation_metadata.get("right_travel_m"),
                "open_sectors": navigation_metadata.get("open_sectors"),
            }
            self.get_logger().info(
                "SCAN context retained for safety only; VLM decides whether "
                "the top-down view is needed"
            )
            self.get_logger().info(
                "Navigation snapshot sequence %s refreshed"
                % navigation_metadata.get("snapshot_sequence")
            )
            with self.lock:
                if self.latest_image is None or self.latest_camera_info is None:
                    raise RuntimeError("camera frame unavailable after snapshot")
                image = self.latest_image.copy()
                header = self.latest_image_header
                camera_info = self.latest_camera_info
            result = self.query(
                image,
                header,
                camera_info,
                navigation_camera_path,
                navigation_topdown_path,
                navigation_prompt_path,
                navigation_metadata,
                query_generation,
                cancel_event,
            )
            query_images = list(result.pop("query_images", []))
            point_selection_images = list(
                result.pop("point_selection_images", query_images)
            )
            query_requests = list(result.pop("query_requests", []))
            system_prompt_name = result.pop(
                "query_system_prompt_name", "unknown"
            )
            navigation_mode = result.get("navigation_mode", "camera")
            result["navigation_context"] = {
                "images": query_images,
                "point_selection_images": point_selection_images,
                "requests": query_requests,
                "prompt": str(navigation_prompt_path),
                "system_prompt": str(
                    self.args.navigation_system_prompt_path
                ),
                "system_prompt_name": system_prompt_name,
                "navigation_mode": navigation_mode,
                "planner_context": planner_context,
                "snapshot_sequence": navigation_metadata.get(
                    "snapshot_sequence"
                ),
                "captured_at": (
                    navigation_metadata.get("snapshot") or {}
                ).get("captured_at"),
                "robot_pose": navigation_metadata.get("robot_pose"),
                "range_m": navigation_metadata.get("range_m"),
            }
            with self.lock:
                superseded = query_generation != self.query_generation
                has_control = bool(result.get("target_xyz"))
                if not superseded and (
                    self.args.lock_target
                    and result.get("ok")
                    and result.get("walkable")
                    and has_control
                ):
                    result["locked"] = True
                    self.locked_result = result
            if superseded:
                self.get_logger().info(
                    "Discarded superseded VLM result after a reselect request"
                )
                return
            if result.get("locked"):
                if result.get("pixel"):
                    self.get_logger().info(
                        "VLM %s target locked at (%d,%d); "
                        "use the RViz reselect button to choose another"
                        % (
                            navigation_mode,
                            result["pixel"][0],
                            result["pixel"][1],
                        )
                    )
                if self.goal_pub is not None:
                    result["approval_required"] = True
                    self.get_logger().info(
                        "Waiting for RViz approval before sending the SCAN goal"
                    )
            self.publish_result(result, image, header)
        except QueryCancelled:
            self.get_logger().info(
                "VLM generation interrupted by reselect"
            )
        except Exception as exc:
            result = {
                "ok": False,
                "error": str(exc),
                "goal_publishing": False,
                "unix_time": time.time(),
            }
            self.status_pub.publish(
                String(data=json.dumps(result, ensure_ascii=False))
            )
            self.get_logger().warning("VLM pixel query failed: %s" % exc)
            self.publish_stream_event(
                {"type": "error", "text": str(exc)}
            )
        finally:
            with self.lock:
                self.busy = False
                if self.active_cancel_event is cancel_event:
                    self.active_cancel_event = None

    def publish_goal(self, result):
        target = result["target_xyz"]
        goal = PoseStamped()
        goal.header.frame_id = self.args.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(target[0])
        goal.pose.position.y = float(target[1])
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.goal_published = True
        result["goal_publishing"] = True
        result["goal_topic"] = self.args.goal_topic
        self.get_logger().info(
            "Published one-shot VLM goal to %s: [%.3f, %.3f, 0.000]"
            % (
                self.args.goal_topic,
                goal.pose.position.x,
                goal.pose.position.y,
            )
        )

    def approval_callback(self, message):
        command = message.data.strip().lower()
        if command == "clear_memory":
            with self.lock:
                self.chat_history.clear()
                self.decision_history_world.clear()
                self.next_request_reason = "operator cleared VLM memory"
                self.last_topdown_mtime_ns = None
            image = cv2.imread(
                str(self.args.topdown_image_path), cv2.IMREAD_COLOR
            )
            if image is not None:
                self.publish_topdown_image(image)
            self.publish_stream_event({"type": "memory_cleared"})
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "ok": True,
                            "memory_cleared": True,
                            "history_messages": 0,
                            "decision_points": 0,
                        }
                    )
                )
            )
            self.get_logger().info(
                "RViz cleared VLM chat memory and decision-map history"
            )
            return

        if command == "reselect":
            with self.lock:
                had_target = self.locked_result is not None
                cancel_event = self.active_cancel_event
                self.locked_result = None
                self.goal_published = False
                self.last_query_at = 0.0
                self.last_topdown_mtime_ns = None
                self.query_generation += 1
                self.next_request_reason = "operator requested a reselect"
                if cancel_event is not None:
                    cancel_event.cancel()
            markers = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.marker_pub.publish(markers)
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "ok": True,
                            "approval_required": False,
                            "reselecting": True,
                            "had_target": had_target,
                        }
                    )
                )
            )
            self.get_logger().info(
                "RViz requested reselect; fresh camera+top-down snapshot "
                "will be captured before the next VLM query"
            )
            return

        if command != "approve":
            self.get_logger().warning(
                "Ignoring unknown VLM approval command: %s" % command
            )
            return

        with self.lock:
            if self.locked_result is None:
                self.get_logger().warning(
                    "Approval ignored: no VLM target is locked"
                )
                return
            if self.goal_published:
                self.get_logger().warning(
                    "Approval ignored: this VLM target was already sent"
                )
                return
            self.locked_result = dict(self.locked_result)
            self.locked_result["approval_required"] = False
            self.locked_result["approved"] = True
            approved_result = dict(self.locked_result)
        self.record_approved_decision(
            approved_result,
            "operator approved",
            {
                "snapshot": {
                    "captured_at": (
                        approved_result.get("navigation_context") or {}
                    ).get("captured_at")
                }
            },
        )
        self.publish_goal(approved_result)
        self.status_pub.publish(
            String(data=json.dumps(approved_result, ensure_ascii=False))
        )

    def prepare_navigation_context(self):
        camera_path = Path(self.args.navigation_camera_image)
        topdown_path = Path(self.args.navigation_topdown_image)
        metadata_path = Path(self.args.navigation_metadata_path)
        script = Path(self.args.navigation_snapshot_script)
        if not script.is_file():
            raise RuntimeError(
                "camera+top-down snapshot script is missing: %s" % script
            )
        completed = subprocess.run(
            [str(script)],
            cwd=str(LOCAL_WS),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=self.args.navigation_snapshot_timeout,
            check=False,
        )
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout).strip()
            raise RuntimeError(
                "camera+top-down snapshot failed: %s" % detail[-600:]
            )

        if not (
            camera_path.is_file()
            and topdown_path.is_file()
            and metadata_path.is_file()
        ):
            raise RuntimeError(
                "camera+top-down snapshot did not produce images and metadata"
            )
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        return (
            camera_path,
            topdown_path,
            metadata,
        )

    def camera_decision_to_result(
        self,
        decision,
        image,
        header,
        camera_info,
        elapsed_ms,
        preliminary=False,
        navigation_metadata=None,
    ):
        """Reuse the original camera-ray and LiDAR-ground back-projection."""
        metadata = navigation_metadata or {}
        if decision.get("map_x") is None or decision.get("map_y") is None:
            raise RuntimeError("VLM camera decision has no image point")
        x_norm = float(decision["map_x"])
        y_norm = float(decision["map_y"])
        if x_norm > 1.0 or y_norm > 1.0:
            x_norm /= max(1, image.shape[1] - 1)
            y_norm /= max(1, image.shape[0] - 1)
        if not (0.0 <= x_norm <= 1.0 and 0.0 <= y_norm <= 1.0):
            raise RuntimeError("VLM camera point is outside the image")
        u = min(
            image.shape[1] - 1,
            max(0, int(round(x_norm * (image.shape[1] - 1)))),
        )
        v = min(
            image.shape[0] - 1,
            max(0, int(round(y_norm * (image.shape[0] - 1)))),
        )

        camera_matrix = np.asarray(
            camera_info.k, dtype=np.float64
        ).reshape(3, 3)
        distortion = np.asarray(camera_info.d, dtype=np.float64).reshape(-1, 1)
        normalized = cv2.fisheye.undistortPoints(
            np.asarray([[[float(u), float(v)]]], dtype=np.float64),
            camera_matrix,
            distortion,
        ).reshape(2)
        ray_camera = np.asarray(
            [normalized[0], normalized[1], 1.0], dtype=np.float64
        )
        ray_camera /= np.linalg.norm(ray_camera)

        camera_frame = header.frame_id or camera_info.header.frame_id
        query_time = Time.from_msg(header.stamp)
        transform, tf_latest = self.lookup_transform(
            camera_frame, query_time
        )
        rotation = quaternion_matrix(transform.transform.rotation)
        camera_origin = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        ray_world = rotation @ ray_camera
        ray_world /= np.linalg.norm(ray_world)

        ground_normal, ground_offset, ground_debug = self.ground_plane()
        denominator = float(np.dot(ground_normal, ray_world))
        if denominator >= -1e-5:
            raise RuntimeError("selected camera ray does not hit the ground")
        ray_length = -(
            float(np.dot(ground_normal, camera_origin)) + ground_offset
        ) / denominator
        if ray_length < self.args.min_camera_target_range:
            raise RuntimeError(
                "selected camera ground point is too close: %.3f m"
                % ray_length
            )
        ground_intersection = camera_origin + ray_length * ray_world
        target, target_debug = self.lidar_target_for_ray(
            camera_origin,
            ray_world,
            ground_normal,
            ground_offset,
            ground_intersection,
            camera_frame,
            query_time,
            camera_info,
            np.asarray([u, v], dtype=np.float64),
        )

        distance_transform, distance_tf_latest = self.lookup_transform(
            self.args.distance_frame, query_time
        )
        robot_origin = np.asarray(
            [
                distance_transform.transform.translation.x,
                distance_transform.transform.translation.y,
                distance_transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        target_distance = math.hypot(
            float(target[0] - robot_origin[0]),
            float(target[1] - robot_origin[1]),
        )
        if target_distance > self.args.max_target_distance:
            raise RuntimeError(
                "selected camera target is too far from dog center: "
                "%.3f m > %.3f m"
                % (target_distance, self.args.max_target_distance)
            )
        bearing_deg = target_bearing_deg(metadata, robot_origin, target)
        action = action_from_bearing(bearing_deg)
        camera_projection = self.reproject_locked_result(
            {"target_xyz": [float(value) for value in target]},
            header,
            camera_info,
        )
        map_pixel = None
        map_pixel_normalized = None
        panel_box = metadata.get("map_panel_box")
        if isinstance(panel_box, (list, tuple)) and len(panel_box) == 4:
            map_size = (
                int(panel_box[2]) - int(panel_box[0]),
                int(panel_box[3]) - int(panel_box[1]),
            )
            map_pixel = world_xyz_to_topdown_pixel(
                target, map_size, metadata
            )
            if map_pixel is not None:
                map_pixel_normalized = [
                    round(map_pixel[0] / max(1, map_size[0] - 1), 5),
                    round(map_pixel[1] / max(1, map_size[1] - 1), 5),
                ]
        result = {
            "ok": True,
            "walkable": True,
            "goal_publishing": False,
            "pixel": [u, v],
            "pixel_source": "camera_ground",
            "pixel_normalized": [round(x_norm, 5), round(y_norm, 5)],
            "camera_pixel": [u, v],
            "camera_visible": bool(camera_projection.get("visible")),
            "map_pixel": map_pixel,
            "map_pixel_normalized": map_pixel_normalized,
            "target_xyz": [round(float(value), 4) for value in target],
            "camera_origin": [
                round(float(value), 4) for value in camera_origin
            ],
            "robot_origin": [
                round(float(value), 4) for value in robot_origin
            ],
            "range_origin": self.args.distance_frame,
            "ray_camera": [round(float(value), 6) for value in ray_camera],
            "ray_world": [round(float(value), 6) for value in ray_world],
            "ground_intersection_xyz": [
                round(float(value), 4) for value in ground_intersection
            ],
            "target_distance_m": round(target_distance, 3),
            "target_source": target_debug["source"],
            "lidar_target": target_debug,
            "ground_plane": ground_debug,
            "target_bearing_deg": round(bearing_deg, 2),
            "recommended_action": action,
            "heading_delta_deg": round(
                normalize_turn_delta_deg(action, bearing_deg), 2
            ),
            "suggested_forward_distance_m": round(target_distance, 3),
            "control_mode": "scan_goal",
            "scene": "vlm_camera_ground",
            "locked": False,
            "confidence": 1.0,
            "reason": "VLM selected a ground point in the front camera.",
            "tf_latest": tf_latest,
            "distance_tf_latest": distance_tf_latest,
            "elapsed_ms": round(elapsed_ms, 1),
        }
        if preliminary:
            result["preliminary"] = True
        return result

    def topdown_decision_to_result(
        self,
        decision,
        topdown_image,
        elapsed_ms,
        preliminary=False,
        navigation_metadata=None,
    ):
        """Convert a VLM top-down floor pixel into a world target."""
        metadata = navigation_metadata or {}
        if decision.get("map_x") is None or decision.get("map_y") is None:
            raise RuntimeError("VLM top-down decision has no map point")
        x_norm = float(decision["map_x"])
        y_norm = float(decision["map_y"])
        snapped = snap_to_nearest_gray_floor(
            topdown_image, x_norm, y_norm, metadata
        )
        if snapped is None:
            raise RuntimeError(
                "VLM top-down target (%.3f, %.3f) is not within "
                "0.2 m of gray floor" % (x_norm, y_norm)
            )
        vlm_x_norm, vlm_y_norm = x_norm, y_norm
        x_norm, y_norm = snapped["x"], snapped["y"]
        target = topdown_pixel_to_world(x_norm, y_norm, metadata)
        robot_pose = metadata.get("robot_pose") or {}
        if not all(key in robot_pose for key in ("x", "y", "z")):
            raise RuntimeError("top-down map has no robot pose")
        robot_origin = np.asarray(
            [
                float(robot_pose["x"]),
                float(robot_pose["y"]),
                float(robot_pose["z"]),
            ],
            dtype=np.float64,
        )
        target_distance = math.hypot(
            float(target[0] - robot_origin[0]),
            float(target[1] - robot_origin[1]),
        )
        if target_distance < 1.20:
            raise RuntimeError(
                "selected top-down target is too close: %.3f m"
                % target_distance
            )
        if target_distance > self.args.max_target_distance:
            raise RuntimeError(
                "selected top-down target is too far: %.3f m > %.3f m"
                % (target_distance, self.args.max_target_distance)
            )

        height, width = topdown_image.shape[:2]
        map_pixel = [
            int(round(x_norm * (width - 1))),
            int(round(y_norm * (height - 1))),
        ]
        bearing_deg = target_bearing_deg(
            metadata, robot_origin, target
        )
        action = action_from_bearing(bearing_deg)
        result = {
            "ok": True,
            "walkable": True,
            "goal_publishing": False,
            "pixel": map_pixel,
            "pixel_source": "topdown_gray_floor",
            "pixel_normalized": [round(x_norm, 5), round(y_norm, 5)],
            "vlm_pixel_normalized": [
                round(vlm_x_norm, 5),
                round(vlm_y_norm, 5),
            ],
            "gray_snap_offset_m": snapped["offset_m"],
            "camera_pixel": None,
            "camera_visible": False,
            "map_pixel": map_pixel,
            "map_pixel_normalized": [
                round(x_norm, 5),
                round(y_norm, 5),
            ],
            "target_xyz": [round(float(value), 4) for value in target],
            "camera_origin": None,
            "robot_origin": [
                round(float(value), 4) for value in robot_origin
            ],
            "range_origin": self.args.distance_frame,
            "ray_camera": None,
            "ray_world": None,
            "ground_intersection_xyz": [
                round(float(value), 4) for value in target
            ],
            "target_distance_m": round(target_distance, 3),
            "target_source": "vlm_topdown_gray_floor",
            "lidar_target": {
                "source": "topdown_gray_floor",
                "ground_support_distance_m": 0.0,
                "ground_support_limit_m": 0.21,
                "ground_support_available": True,
            },
            "ground_plane": {
                "source": "topdown",
                "z": round(
                    float(robot_pose["z"])
                    + float(metadata["ground_z_m"]),
                    4,
                ),
            },
            "target_bearing_deg": round(bearing_deg, 2),
            "recommended_action": action,
            "heading_delta_deg": round(
                normalize_turn_delta_deg(action, bearing_deg), 2
            ),
            "suggested_forward_distance_m": round(target_distance, 3),
            "control_mode": "scan_goal",
            "scene": "vlm_topdown_wall_fallback",
            "locked": False,
            "confidence": 1.0,
            "reason": (
                "Camera was blocked near a wall; VLM selected a gray "
                "walkable floor point from the top-down map."
            ),
            "tf_latest": True,
            "distance_tf_latest": True,
            "elapsed_ms": round(elapsed_ms, 1),
        }
        if preliminary:
            result["preliminary"] = True
        return result

    def query(
        self,
        image,
        header,
        camera_info,
        navigation_camera_path,
        navigation_topdown_path,
        navigation_prompt_path,
        navigation_metadata,
        query_generation,
        cancel_event,
    ):
        query_camera = annotate_query_image(image)
        cv2.imwrite(
            str(navigation_camera_path),
            query_camera,
            [cv2.IMWRITE_JPEG_QUALITY, 90],
        )
        cv2.imwrite(
            self.args.query_image_path,
            query_camera,
            [cv2.IMWRITE_JPEG_QUALITY, 90],
        )
        system_prompt_path = Path(
            self.args.navigation_system_prompt_path
        )
        system_prompt_path.parent.mkdir(parents=True, exist_ok=True)
        history_messages = self.chat_history_snapshot()
        query_requests = []

        def run_request(
            navigation_mode,
            system_prompt_name,
            system_prompt,
            prompt,
            encoded_images,
            image_paths,
            project_decision,
        ):
            system_prompt_path.write_text(
                system_prompt + "\n", encoding="utf-8"
            )
            navigation_prompt_path.write_text(
                prompt + "\n", encoding="utf-8"
            )
            request_started = time.monotonic()
            self.publish_stream_event(
                {
                    "type": "reset",
                    "unix_time": time.time(),
                    "images": list(image_paths),
                    "navigation_mode": navigation_mode,
                    "system_prompt_name": system_prompt_name,
                }
            )
            query_requests.append(
                {
                    "mode": navigation_mode,
                    "images": list(image_paths),
                    "system_prompt_name": system_prompt_name,
                }
            )
            parser = StreamValueParser()
            visible_stream = DecisionStreamFilter()
            quick_published = False

            def stream_callback(text, channel):
                nonlocal quick_published
                if cancel_event.is_set():
                    raise QueryCancelled("VLM generation cancelled")
                if channel in ("content", "reasoning") and text:
                    visible = (
                        visible_stream.feed(text)
                        if channel == "content"
                        else ""
                    )
                    if visible:
                        self.publish_stream_event(
                            {
                                "type": "delta",
                                "channel": "content",
                                "text": visible,
                            }
                        )
                elif channel == "final":
                    visible = visible_stream.feed("", final=True)
                    if visible:
                        self.publish_stream_event(
                            {
                                "type": "delta",
                                "channel": "content",
                                "text": visible,
                            }
                        )
                if channel not in ("content", "final") or quick_published:
                    return
                values = parser.feed(text, final=channel == "final")
                if values.get("navigation_view") == "topdown":
                    quick_published = True
                    self.publish_stream_event(
                        {
                            "type": "route_decision",
                            "navigation_mode": "topdown",
                            "source": "vlm_camera",
                        }
                    )
                    return
                if (
                    values.get("map_x") is None
                    or values.get("map_y") is None
                ):
                    return
                if project_decision is None:
                    return
                quick_published = True
                elapsed_ms = (
                    time.monotonic() - request_started
                ) * 1000.0
                try:
                    quick_result = project_decision(
                        values, elapsed_ms, True
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        "Early VLM coordinate projection failed: %s" % exc
                    )
                    return
                with self.lock:
                    if (
                        cancel_event.is_set()
                        or query_generation != self.query_generation
                    ):
                        raise QueryCancelled("VLM generation superseded")
                    has_control = bool(quick_result.get("target_xyz"))
                    if self.args.lock_target and has_control:
                        self.locked_result = quick_result
                self.publish_result(quick_result, query_camera, header)
                self.publish_stream_event(
                    {
                        "type": "coords",
                        "elapsed_ms": round(elapsed_ms, 1),
                        "pixel": quick_result.get("pixel"),
                        "target_xyz": quick_result.get("target_xyz"),
                        "action": quick_result.get(
                            "recommended_action"
                        ),
                        "heading_delta_deg": quick_result.get(
                            "heading_delta_deg"
                        ),
                        "target_distance_m": quick_result.get(
                            "target_distance_m"
                        ),
                        "navigation_mode": navigation_mode,
                    }
                )
                self.get_logger().info(
                    "Early VLM %s point displayed after %.0f ms at %s"
                    % (
                        navigation_mode,
                        elapsed_ms,
                        quick_result.get("pixel"),
                    )
                )

            decision = query_json(
                self.args,
                prompt,
                encoded_images,
                system_prompt=system_prompt,
                history_messages=history_messages,
                stream_callback=stream_callback,
                cancel_event=cancel_event,
            )
            elapsed_ms = (
                time.monotonic() - request_started
            ) * 1000.0
            self.publish_stream_event(
                {
                    "type": "done",
                    "elapsed_ms": round(elapsed_ms, 1),
                    "navigation_mode": navigation_mode,
                    "decision": decision,
                }
            )
            if decision.get("navigation_view") == "topdown":
                return decision, None
            if project_decision is None:
                raise RuntimeError(
                    "VLM request has no projection handler"
                )
            if (
                decision.get("map_x") is None
                or decision.get("map_y") is None
            ):
                raise RuntimeError(
                    "VLM response contains no camera point"
                )
            return decision, project_decision(
                decision, elapsed_ms, False
            )

        encoded_camera = cv2.imencode(
            ".jpg", query_camera, [cv2.IMWRITE_JPEG_QUALITY, 82]
        )[1].tobytes()
        camera_path = str(navigation_camera_path)
        topdown_path = str(navigation_topdown_path)

        def project_camera(values, elapsed_ms, preliminary):
            return self.camera_decision_to_result(
                values,
                image,
                header,
                camera_info,
                elapsed_ms,
                preliminary=preliminary,
                navigation_metadata=navigation_metadata,
            )

        camera_fallback_reason = ""
        try:
            camera_decision, result = run_request(
                "camera",
                "CAMERA_ROUTER_SYSTEM_PROMPT",
                CAMERA_ROUTER_SYSTEM_PROMPT,
                CAMERA_USER_PROMPT,
                [encoded_camera],
                [camera_path],
                project_camera,
            )
        except RuntimeError as exc:
            message = str(exc)
            if not any(
                marker in message
                for marker in (
                    "camera point",
                    "camera target",
                    "camera ray",
                    "LiDAR",
                    "ground",
                )
            ):
                raise
            camera_decision = {"navigation_view": "topdown"}
            result = None
            camera_fallback_reason = message
            self.get_logger().warning(
                "Camera floor projection failed; trying top-down "
                "localization: %s" % message
            )
        if camera_decision.get("navigation_view") != "topdown":
            if result is None:
                raise RuntimeError(
                    "VLM camera request returned no floor point"
                )
            result["navigation_mode"] = "camera"
            result["query_images"] = [camera_path]
            result["point_selection_images"] = [camera_path]
            result["query_requests"] = query_requests
            result["query_system_prompt_name"] = (
                "CAMERA_ROUTER_SYSTEM_PROMPT"
            )
            return result

        clean_topdown = cv2.imread(
            str(navigation_topdown_path), cv2.IMREAD_COLOR
        )
        if clean_topdown is None:
            raise RuntimeError(
                "cannot read top-down snapshot: %s"
                % navigation_topdown_path
            )
        query_topdown = annotate_topdown_query(
            clean_topdown,
            float(navigation_metadata.get("range_m", 5.0)),
        )
        query_topdown = overlay_decision_history(
            query_topdown,
            navigation_metadata,
            self.decision_history_snapshot(),
        )
        encoded_topdown = cv2.imencode(
            ".jpg", query_topdown, [cv2.IMWRITE_JPEG_QUALITY, 82]
        )[1].tobytes()

        def project_topdown(values, elapsed_ms, preliminary):
            return self.topdown_decision_to_result(
                values,
                clean_topdown,
                elapsed_ms,
                preliminary=preliminary,
                navigation_metadata=navigation_metadata,
            )

        topdown_decision, result = run_request(
            "topdown_localization",
            "TOPDOWN_LOCALIZATION_SYSTEM_PROMPT",
            TOPDOWN_LOCALIZATION_SYSTEM_PROMPT,
            "The camera is blocked by a nearby wall and no floor is visible. "
            "Choose one gray floor point from the top-down map.",
            [encoded_camera, encoded_topdown],
            [camera_path, topdown_path],
            project_topdown,
        )
        if result is None:
            raise RuntimeError(
                "VLM returned no top-down floor point"
            )
        result["navigation_mode"] = "topdown"
        result["fallback_from_camera"] = True
        result["camera_fallback_reason"] = (
            camera_fallback_reason or "VLM requested topdown"
        )
        result["query_images"] = [camera_path, topdown_path]
        result["point_selection_images"] = [topdown_path]
        result["query_requests"] = query_requests
        result["query_system_prompt_name"] = (
            "TOPDOWN_LOCALIZATION_SYSTEM_PROMPT"
        )
        return result

    def lookup_transform(self, source_frame, stamp):
        tf_latest = False
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.frame_id,
                source_frame,
                stamp,
                timeout=Duration(seconds=0.3),
            )
        except TransformException:
            transform = self.tf_buffer.lookup_transform(
                self.args.frame_id,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            tf_latest = True
        return transform, tf_latest

    def lidar_target_for_ray(
        self,
        origin,
        ray_world,
        ground_normal,
        ground_offset,
        ground_intersection,
        camera_frame,
        query_time,
        camera_info,
        selected_pixel,
    ):
        with self.lock:
            cloud = self.latest_cloud
            cloud_header = self.latest_cloud_header
        if cloud is None or cloud_header is None:
            raise RuntimeError("no LiDAR cloud available for target selection")

        points = np.asarray(
            list(
                point_cloud2.read_points(
                    cloud,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            ),
            dtype=np.float64,
        )
        if points.ndim != 2 or points.shape[0] < 50:
            raise RuntimeError("LiDAR cloud is too sparse for target selection")

        transform, tf_latest = self.lookup_transform(
            cloud_header.frame_id, Time.from_msg(cloud_header.stamp)
        )
        rotation = quaternion_matrix(transform.transform.rotation)
        cloud_origin = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        points_world = points @ rotation.T + cloud_origin
        floor_error = np.abs(
            points_world @ ground_normal + ground_offset
        )
        floor_indices = np.flatnonzero(floor_error <= 0.15)

        try:
            world_to_camera = self.tf_buffer.lookup_transform(
                camera_frame,
                self.args.frame_id,
                query_time,
                timeout=Duration(seconds=0.3),
            )
            world_to_camera_latest = False
        except TransformException:
            world_to_camera = self.tf_buffer.lookup_transform(
                camera_frame,
                self.args.frame_id,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            world_to_camera_latest = True
        camera_rotation = quaternion_matrix(
            world_to_camera.transform.rotation
        )
        camera_translation = np.array(
            [
                world_to_camera.transform.translation.x,
                world_to_camera.transform.translation.y,
                world_to_camera.transform.translation.z,
            ],
            dtype=np.float64,
        )
        points_camera = (
            points_world @ camera_rotation.T + camera_translation
        )
        camera_matrix = np.asarray(
            camera_info.k, dtype=np.float64
        ).reshape(3, 3)
        distortion = np.asarray(
            camera_info.d, dtype=np.float64
        ).reshape(-1, 1)
        pixel_error = np.full(len(points_world), np.inf, dtype=np.float64)
        forward = points_camera[:, 2] > 0.05
        if np.any(forward):
            projected, _ = cv2.fisheye.projectPoints(
                points_camera[forward].reshape(-1, 1, 3),
                np.zeros((3, 1), dtype=np.float64),
                np.zeros((3, 1), dtype=np.float64),
                camera_matrix,
                distortion,
            )
            projected = projected.reshape(-1, 2)
            finite = np.all(np.isfinite(projected), axis=1)
            finite_indices = np.flatnonzero(forward)[finite]
            pixel_error[finite_indices] = np.linalg.norm(
                projected[finite] - selected_pixel[np.newaxis, :],
                axis=1,
            )

        support_distance = None
        matched_floor_indices = floor_indices[
            pixel_error[floor_indices] <= 20.0
        ]
        if matched_floor_indices.size:
            support_distance = float(
                np.min(
                    np.linalg.norm(
                        points_world[matched_floor_indices]
                        - ground_intersection,
                        axis=1,
                    )
                )
            )
        ground_horizontal_range = math.hypot(
            float(ground_intersection[0] - origin[0]),
            float(ground_intersection[1] - origin[1]),
        )
        support_limit = max(0.35, 0.20 * ground_horizontal_range)
        supported = (
            support_distance is not None and support_distance <= support_limit
        )
        debug = {
            "source": (
                "camera_ground_intersection"
                if supported
                else "camera_ground_intersection_sparse_lidar"
            ),
            "ground_support_distance_m": (
                None
                if support_distance is None
                else round(support_distance, 3)
            ),
            "ground_support_limit_m": round(support_limit, 3),
            "ground_support_available": supported,
            "support_points": int(matched_floor_indices.size),
            "support_pixel_error_px": (
                None
                if not matched_floor_indices.size
                else round(
                    float(np.min(pixel_error[matched_floor_indices])), 3
                )
            ),
            "cloud_points": int(len(points)),
            "cloud_age_s": round(
                max(
                    0.0,
                    self.get_clock().now().nanoseconds * 1e-9
                    - Time.from_msg(cloud_header.stamp).nanoseconds * 1e-9,
                ),
                3,
            ),
            "tf_latest": tf_latest,
            "camera_tf_latest": world_to_camera_latest,
        }
        return ground_intersection, debug

    def reproject_locked_result(self, result, header, camera_info):
        target_world = np.asarray(result["target_xyz"], dtype=np.float64)
        camera_frame = header.frame_id or camera_info.header.frame_id
        query_time = Time.from_msg(header.stamp)
        tf_latest = False
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                self.args.frame_id,
                query_time,
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                self.args.frame_id,
                Time(),
                timeout=Duration(seconds=0.15),
            )
            tf_latest = True

        rotation = quaternion_matrix(transform.transform.rotation)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        target_camera = rotation @ target_world + translation
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                self.args.frame_id,
                self.args.distance_frame,
                query_time,
                timeout=Duration(seconds=0.05),
            )
            robot_tf_latest = False
        except TransformException:
            robot_transform = self.tf_buffer.lookup_transform(
                self.args.frame_id,
                self.args.distance_frame,
                Time(),
                timeout=Duration(seconds=0.15),
            )
            robot_tf_latest = True
        robot_origin = np.array(
            [
                robot_transform.transform.translation.x,
                robot_transform.transform.translation.y,
                robot_transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        target_from_robot = target_world - robot_origin
        target_from_camera = target_camera
        projected = dict(result)
        projected["reprojected"] = True
        projected["reprojection_tf_latest"] = tf_latest
        projected["robot_origin"] = [
            round(float(value), 4) for value in robot_origin
        ]
        projected["range_origin"] = self.args.distance_frame
        projected["target_distance_m"] = round(
            float(np.linalg.norm(target_from_robot[:2])), 3
        )
        projected["target_slant_range_m"] = round(
            float(np.linalg.norm(target_from_robot)), 3
        )
        projected["camera_target_distance_m"] = round(
            float(np.linalg.norm(target_from_camera[:2])), 3
        )
        projected["camera_target_slant_range_m"] = round(
            float(np.linalg.norm(target_from_camera)), 3
        )
        projected["distance_tf_latest"] = robot_tf_latest
        projected["visible"] = False
        projected["camera_pixel"] = None
        if target_camera[2] <= 0.05:
            return projected

        camera_matrix = np.asarray(
            camera_info.k, dtype=np.float64
        ).reshape(3, 3)
        distortion = np.asarray(
            camera_info.d, dtype=np.float64
        ).reshape(-1, 1)
        image_point, _ = cv2.fisheye.projectPoints(
            target_camera.reshape(1, 1, 3),
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            camera_matrix,
            distortion,
        )
        u, v = image_point.reshape(2)
        if not np.isfinite(u) or not np.isfinite(v):
            return projected
        u_int = int(round(float(u)))
        v_int = int(round(float(v)))
        projected["camera_pixel"] = [u_int, v_int]
        projected["visible"] = (
            0 <= u_int < camera_info.width
            and 0 <= v_int < camera_info.height
        )
        return projected

    def ground_plane(self):
        if self.args.ground_z is not None:
            return (
                np.array([0.0, 0.0, 1.0], dtype=np.float64),
                -float(self.args.ground_z),
                {
                    "source": "argument",
                    "z": round(float(self.args.ground_z), 4),
                    "normal": [0.0, 0.0, 1.0],
                },
            )

        with self.lock:
            cloud = self.latest_cloud
            cloud_header = self.latest_cloud_header
        if cloud is None or cloud_header is None:
            raise RuntimeError("no LiDAR cloud available for ground estimation")
        try:
            points = np.asarray(
                list(
                    point_cloud2.read_points(
                        cloud,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ),
                dtype=np.float64,
            )
        except Exception as exc:
            raise RuntimeError("cannot read LiDAR cloud: %s" % exc)
        if points.ndim != 2 or points.shape[0] < 50:
            raise RuntimeError("LiDAR cloud is too sparse for ground estimation")

        transform, tf_latest = self.lookup_transform(
            cloud_header.frame_id, Time.from_msg(cloud_header.stamp)
        )
        rotation = quaternion_matrix(transform.transform.rotation)
        origin = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        points_world = points @ rotation.T + origin
        radius = np.hypot(
            points_world[:, 0] - origin[0],
            points_world[:, 1] - origin[1],
        )
        ground_candidates = points_world[
            (radius > 0.45)
            & (radius < 6.0)
            & (points_world[:, 2] < origin[2] - 0.15)
        ]
        if len(ground_candidates) < 30:
            raise RuntimeError("not enough nearby points to estimate the ground")

        z_values = ground_candidates[:, 2]
        lower = min(float(np.percentile(z_values, 1)) - 0.04, origin[2] - 1.2)
        upper = float(origin[2] - 0.15)
        bins = np.arange(lower, upper + 0.0201, 0.02)
        counts, edges = np.histogram(z_values, bins=bins)
        if len(counts) < 9:
            raise RuntimeError("LiDAR ground histogram is too short")
        smoothed = np.convolve(counts, np.ones(5, dtype=np.int64), mode="same")
        minimum = max(25, int(0.40 * float(np.max(smoothed))))
        peak_indices = [
            index
            for index in range(2, len(smoothed) - 2)
            if smoothed[index] >= minimum
            and smoothed[index] >= smoothed[index - 2]
            and smoothed[index] >= smoothed[index + 2]
        ]
        if not peak_indices:
            raise RuntimeError("LiDAR ground histogram has no stable peak")
        groups = [[peak_indices[0]]]
        for index in peak_indices[1:]:
            if float(edges[index] - edges[groups[-1][-1]]) <= 0.10:
                groups[-1].append(index)
            else:
                groups.append([index])
        floor_group = min(
            groups, key=lambda group: float(np.mean([edges[index] for index in group]))
        )
        group_peak = max(floor_group, key=lambda index: int(smoothed[index]))
        run_low = float(edges[group_peak]) - 0.045
        run_high = float(edges[group_peak]) + 0.045
        inliers = ground_candidates[
            (ground_candidates[:, 2] >= run_low)
            & (ground_candidates[:, 2] <= run_high)
        ]
        required_inliers = max(12, int(0.015 * len(points)))
        if len(inliers) < required_inliers:
            run_low = float(edges[group_peak]) - 0.08
            run_high = float(edges[group_peak]) + 0.08
            inliers = ground_candidates[
                (ground_candidates[:, 2] >= run_low)
                & (ground_candidates[:, 2] <= run_high)
            ]
        if len(inliers) < required_inliers:
            raise RuntimeError("LiDAR ground fit has too few inliers")

        design = np.column_stack(
            (inliers[:, 0], inliers[:, 1], np.ones(len(inliers)))
        )
        slope_x, slope_y, intercept = np.linalg.lstsq(
            design, inliers[:, 2], rcond=None
        )[0]
        normal_raw = np.array(
            [-slope_x, -slope_y, 1.0], dtype=np.float64
        )
        normal_norm = float(np.linalg.norm(normal_raw))
        normal = normal_raw / normal_norm
        offset = -float(intercept) / normal_norm
        ground_z = float(
            np.median(
                slope_x * inliers[:, 0]
                + slope_y * inliers[:, 1]
                + intercept
            )
        )
        if abs(float(normal[2])) < 0.98:
            normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            ground_z = float(np.median(inliers[:, 2]))
            offset = -ground_z
        return (
            normal,
            offset,
            {
                "source": "lidar",
                "z": round(ground_z, 4),
                "normal": [round(float(value), 5) for value in normal],
                "inliers": int(len(inliers)),
                "cloud_points": int(len(points)),
                "tf_latest": tf_latest,
            },
        )

    def publish_result(self, result, image, header, republish=False):
        if not republish:
            self.status_pub.publish(
                String(data=json.dumps(result, ensure_ascii=False))
            )
            self.publish_topdown_result(result)
        if not result.get("ok"):
            return

        annotated = image.copy()
        if not result.get("walkable"):
            message = "NO SAFE FORWARD ROUTE"
            box_width = min(image.shape[1] - 24, 430)
            box_left = (image.shape[1] - box_width) // 2
            cv2.rectangle(
                annotated,
                (box_left, 18),
                (box_left + box_width, 74),
                (18, 18, 18),
                -1,
            )
            cv2.rectangle(
                annotated,
                (box_left, 18),
                (box_left + box_width, 74),
                (35, 35, 255),
                2,
            )
            cv2.putText(
                annotated,
                message,
                (box_left + 18, 54),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        elif result.get("camera_pixel"):
            u, v = result["camera_pixel"]
            orange = (0, 165, 255)
            red = (35, 35, 255)
            halo = annotated.copy()
            cv2.circle(halo, (u, v), 34, orange, -1, cv2.LINE_AA)
            cv2.addWeighted(halo, 0.18, annotated, 0.82, 0.0, annotated)
            cv2.circle(annotated, (u, v), 30, (15, 15, 15), 8, cv2.LINE_AA)
            cv2.circle(annotated, (u, v), 30, orange, 4, cv2.LINE_AA)
            cv2.circle(annotated, (u, v), 17, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.circle(annotated, (u, v), 6, red, -1, cv2.LINE_AA)
            cv2.line(
                annotated,
                (u - 42, v),
                (u - 20, v),
                (255, 255, 255),
                3,
                cv2.LINE_AA,
            )
            cv2.line(
                annotated,
                (u + 20, v),
                (u + 42, v),
                (255, 255, 255),
                3,
                cv2.LINE_AA,
            )
            cv2.line(
                annotated,
                (u, v - 42),
                (u, v - 20),
                (255, 255, 255),
                3,
                cv2.LINE_AA,
            )
            cv2.line(
                annotated,
                (u, v + 20),
                (u, v + 42),
                (255, 255, 255),
                3,
                cv2.LINE_AA,
            )
            distance = float(result.get("target_distance_m", 0.0))
            confidence = 100.0 * float(result.get("confidence", 0.0))
            label = (
                "VLM CAMERA GROUND"
                if result.get("pixel_source") == "camera_ground"
                else "VLM TOP-DOWN GRAY"
            )
            detail = "cam (%d,%d)  DOG %.2f m  %.0f%%" % (
                u,
                v,
                distance,
                confidence,
            )
            label_x = max(12, min(image.shape[1] - 360, u + 46))
            label_y = max(60, min(image.shape[0] - 76, v - 82))
            cv2.rectangle(
                annotated,
                (label_x, label_y),
                (label_x + 336, label_y + 62),
                (18, 18, 18),
                -1,
            )
            cv2.rectangle(
                annotated,
                (label_x, label_y),
                (label_x + 336, label_y + 62),
                orange,
                2,
            )
            cv2.circle(
                annotated,
                (label_x + 20, label_y + 20),
                6,
                red,
                -1,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                label,
                (label_x + 34, label_y + 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                detail,
                (label_x + 14, label_y + 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                orange,
                2,
                cv2.LINE_AA,
            )
        else:
            distance = float(result.get("target_distance_m", 0.0))
            map_pixel = result.get("map_pixel")
            message = "SCAN GOAL FROM TOP-DOWN  %.2f m" % distance
            if map_pixel:
                message = "MAP (%d,%d) -> SCAN GOAL  %.2f m" % (
                    int(map_pixel[0]),
                    int(map_pixel[1]),
                    distance,
                )
            box_width = min(image.shape[1] - 24, 430)
            box_left = (image.shape[1] - box_width) // 2
            cv2.rectangle(
                annotated,
                (box_left, 18),
                (box_left + box_width, 74),
                (18, 18, 18),
                -1,
            )
            cv2.rectangle(
                annotated,
                (box_left, 18),
                (box_left + box_width, 74),
                (0, 165, 255),
                2,
            )
            cv2.putText(
                annotated,
                message,
                (box_left + 18, 54),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        self.image_pub.publish(bgr_to_image_message(annotated, header))

        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.args.frame_id
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if not result.get("walkable") or not result.get("target_xyz"):
            self.marker_pub.publish(markers)
            return

        target = result["target_xyz"]
        origin = result.get("robot_origin") or [0.0, 0.0, 0.0]
        stamp = self.get_clock().now().to_msg()
        ray = Marker()
        ray.header.frame_id = self.args.frame_id
        ray.header.stamp = stamp
        ray.ns = "vlm_pixel_ray"
        ray.id = 1
        ray.type = Marker.LINE_LIST
        ray.action = Marker.ADD
        ray.pose.orientation.w = 1.0
        ray.scale.x = 0.025
        ray.color.r, ray.color.g, ray.color.b, ray.color.a = (
            1.0,
            0.75,
            0.05,
            0.95,
        )
        from geometry_msgs.msg import Point

        ray.points = [Point(x=origin[0], y=origin[1], z=origin[2]),
                      Point(x=target[0], y=target[1], z=target[2])]
        markers.markers.append(ray)

        base = Marker()
        base.header.frame_id = self.args.frame_id
        base.header.stamp = stamp
        base.ns = "vlm_pixel_target"
        base.id = 2
        base.type = Marker.CYLINDER
        base.action = Marker.ADD
        base.pose.position.x = target[0]
        base.pose.position.y = target[1]
        base.pose.position.z = target[2] + 0.015
        base.pose.orientation.w = 1.0
        base.scale.x = base.scale.y = 0.24
        base.scale.z = 0.03
        base.color.r, base.color.g, base.color.b, base.color.a = (
            1.0,
            0.05,
            0.03,
            0.95,
        )
        markers.markers.append(base)

        stem = Marker()
        stem.header.frame_id = self.args.frame_id
        stem.header.stamp = stamp
        stem.ns = "vlm_pixel_target"
        stem.id = 3
        stem.type = Marker.CYLINDER
        stem.action = Marker.ADD
        stem.pose.position.x = target[0]
        stem.pose.position.y = target[1]
        stem.pose.position.z = target[2] + 0.4
        stem.pose.orientation.w = 1.0
        stem.scale.x = stem.scale.y = 0.06
        stem.scale.z = 0.8
        stem.color.r, stem.color.g, stem.color.b, stem.color.a = (
            1.0,
            0.05,
            0.03,
            0.95,
        )
        markers.markers.append(stem)

        head = Marker()
        head.header.frame_id = self.args.frame_id
        head.header.stamp = stamp
        head.ns = "vlm_pixel_target"
        head.id = 4
        head.type = Marker.SPHERE
        head.action = Marker.ADD
        head.pose.position.x = target[0]
        head.pose.position.y = target[1]
        head.pose.position.z = target[2] + 0.86
        head.pose.orientation.w = 1.0
        head.scale.x = head.scale.y = head.scale.z = 0.22
        head.color.r, head.color.g, head.color.b, head.color.a = (
            1.0,
            0.05,
            0.03,
            0.95,
        )
        markers.markers.append(head)

        label = Marker()
        label.header.frame_id = self.args.frame_id
        label.header.stamp = stamp
        label.ns = "vlm_pixel_target"
        label.id = 5
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = target[0]
        label.pose.position.y = target[1]
        label.pose.position.z = target[2] + 1.08
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color.r, label.color.g, label.color.b, label.color.a = (
            1.0,
            0.2,
            0.1,
            0.98,
        )
        if result.get("pixel"):
            label.text = "VLM (%d,%d) DOG %.2fm" % (
                result["pixel"][0],
                result["pixel"][1],
                result["target_distance_m"],
            )
        else:
            label.text = "VLM TARGET DOG %.2fm" % result[
                "target_distance_m"
            ]
        markers.markers.append(label)
        if not republish:
            self.marker_pub.publish(markers)
            self.get_logger().info(json.dumps(result, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image-topic", default="/go2/front_camera/image_raw"
    )
    parser.add_argument(
        "--camera-info-topic", default="/go2/front_camera/camera_info"
    )
    parser.add_argument(
        "--marker-topic", default="/vlm_pixel_shadow"
    )
    parser.add_argument(
        "--ground-cloud-topic", default="/LIO/clouds_lidar"
    )
    parser.add_argument(
        "--distance-frame",
        default="base",
        help="robot-center frame used for the displayed target range",
    )
    parser.add_argument(
        "--image-output-topic", default="/go2/front_camera/vlm_point_overlay"
    )
    parser.add_argument(
        "--status-topic", default="/vlm_pixel_shadow/status"
    )
    parser.add_argument("--frame-id", default="world")
    parser.add_argument(
        "--ground-z",
        type=float,
        default=None,
        help="override the LiDAR-estimated ground height in the world frame",
    )
    parser.add_argument("--max-range", type=float, default=50.0)
    parser.add_argument(
        "--max-target-distance",
        type=float,
        default=4.5,
        help="maximum accepted horizontal target range from the distance frame",
    )
    parser.add_argument(
        "--min-camera-target-range",
        type=float,
        default=0.5,
        help="minimum camera-ray ground target range",
    )
    parser.add_argument("--interval", type=float, default=8.0)
    parser.add_argument(
        "--drive",
        action="store_true",
        help="publish one goal only after RViz approval of the locked target",
    )
    parser.add_argument(
        "--goal-topic", default="/move_base_simple/goal"
    )
    parser.add_argument(
        "--approval-topic", default="/go2/vlm_approval"
    )
    parser.add_argument(
        "--lock-target", dest="lock_target", action="store_true"
    )
    parser.add_argument(
        "--no-lock-target", dest="lock_target", action="store_false"
    )
    parser.set_defaults(lock_target=True, drive=False)
    parser.add_argument(
        "--query-image-path", default="/tmp/vlm_pixel_query.jpg"
    )
    parser.add_argument(
        "--navigation-snapshot-script",
        default=str(
            LOCAL_WS / "debug/tools/capture_vlm_navigation_snapshot.sh"
        ),
    )
    parser.add_argument(
        "--navigation-camera-image",
        default=str(
            LOCAL_WS / "debug/pointcloud_views/front_camera_current.jpg"
        ),
    )
    parser.add_argument(
        "--navigation-topdown-image",
        default=str(
            LOCAL_WS / "debug/pointcloud_views/mid360_topdown_5s.png"
        ),
    )
    parser.add_argument(
        "--navigation-prompt-path",
        default=str(
            LOCAL_WS
            / "debug/pointcloud_views/vlm_navigation_current.txt"
        ),
    )
    parser.add_argument(
        "--navigation-system-prompt-path",
        default=str(
            LOCAL_WS
            / "debug/pointcloud_views/vlm_navigation_system.txt"
        ),
    )
    parser.add_argument(
        "--navigation-metadata-path",
        default=str(
            LOCAL_WS
            / "debug/pointcloud_views/vlm_navigation_current.json"
        ),
    )
    parser.add_argument(
        "--topdown-image-path",
        default=str(
            LOCAL_WS / "debug/pointcloud_views/mid360_topdown_5s.png"
        ),
    )
    parser.add_argument(
        "--topdown-topic", default="/vlm_navigation/topdown"
    )
    parser.add_argument(
        "--stream-topic", default="/vlm_pixel_shadow/stream"
    )
    parser.add_argument(
        "--navigation-snapshot-timeout",
        type=float,
        default=60.0,
    )
    parser.add_argument(
        "--api-url",
        default="http://127.0.0.1:8001/v1/chat/completions",
    )
    parser.add_argument("--model", default="unsloth/Qwen3.5-27B-GGUF")
    parser.add_argument(
        "--reasoning-effort",
        choices=("none", "low", "medium", "high"),
        default="low",
    )
    parser.add_argument(
        "--disable-thinking",
        action="store_true",
        help="disable template-level reasoning for local models",
    )
    parser.add_argument(
        "--max-output-tokens",
        type=int,
        default=128,
        help="maximum reasoning plus coordinate tokens per VLM request",
    )
    parser.add_argument("--api-timeout", type=float, default=60.0)
    parser.add_argument(
        "--env-file", default=os.path.expanduser("~/.env")
    )
    args = parser.parse_args()
    if (
        args.interval <= 0.0
        or args.max_range <= 0.1
        or args.max_target_distance <= 0.0
        or args.max_target_distance >= args.max_range
        or args.min_camera_target_range <= 0.0
        or args.min_camera_target_range >= args.max_target_distance
        or args.max_output_tokens < 16
    ):
        parser.error("target range settings are invalid")
    load_dotenv(args.env_file)
    rclpy.init()
    node = VlmPixelShadow(args)
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

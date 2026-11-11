#!/usr/bin/env python3
"""Ask a VLM for one floor pixel and visualize its metric ground projection."""

import argparse
import json
import math
import os
import threading
import time

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

from vlm_common import load_dotenv, query_json


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
        if locked_result is not None and camera_info is not None:
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
                locked_result["pixel"] = None
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
            image = self.latest_image.copy()
            header = self.latest_image_header
            camera_info = self.latest_camera_info
            self.busy = True
            self.last_query_at = now
        threading.Thread(
            target=self.query_worker,
            args=(image, header, camera_info),
            name="vlm-pixel-query",
            daemon=True,
        ).start()

    def query_worker(self, image, header, camera_info):
        try:
            result = self.query(image, header, camera_info)
            if (
                self.args.lock_target
                and result.get("ok")
                and result.get("walkable")
                and result.get("target_xyz")
            ):
                result["locked"] = True
                with self.lock:
                    self.locked_result = result
                self.get_logger().info(
                    "VLM floor target locked at pixel (%d,%d); use the RViz "
                    "reselect button to select a new target"
                    % tuple(result["pixel"])
                )
                if self.goal_pub is not None:
                    result["approval_required"] = True
                    self.get_logger().info(
                        "Waiting for RViz approval before publishing the goal"
                    )
            self.publish_result(result, image, header)
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
        finally:
            with self.lock:
                self.busy = False

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
        if command == "reselect":
            with self.lock:
                had_candidate = self.locked_result is not None
                self.locked_result = None
                self.goal_published = False
                self.last_query_at = 0.0
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
                            "had_candidate": had_candidate,
                        }
                    )
                )
            )
            self.get_logger().info(
                "RViz requested reselect; clearing the locked VLM target"
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
                    "Approval ignored: no VLM candidate is locked"
                )
                return
            if self.goal_published:
                self.get_logger().warning(
                    "Approval ignored: this VLM candidate was already sent"
                )
                return
            self.locked_result = dict(self.locked_result)
            self.locked_result["approval_required"] = False
            self.locked_result["approved"] = True
            approved_result = dict(self.locked_result)
        self.publish_goal(approved_result)
        self.status_pub.publish(
            String(data=json.dumps(approved_result, ensure_ascii=False))
        )

    def query(self, image, header, camera_info):
        query_image = annotate_query_image(image)
        cv2.imwrite(
            self.args.query_image_path,
            query_image,
            [cv2.IMWRITE_JPEG_QUALITY, 90],
        )
        prompt = (
            "你在看室内四足机器人的前视鱼眼相机。根据图像中的地面、墙脚、"
            "家具和障碍物，识别一条从画面近处连续延伸的可通行通道，并估计"
            "通道横向中心线。选点必须靠近通道中心，远离人物、家具、桌椅腿"
            "和两侧边界；不要选地平线、远端未知区域、墙面或门后。优先选择"
            "清晰可见的中近景地面点，目标距机器人约3到5.5米，不要刻意选最远点。"
            "图中的灰色网格是归一化坐标，"
            "x向右、y向下，均从0到1。返回该地面点的归一化坐标，"
            "不要返回世界坐标或距离。严格只输出JSON："
            '{"walkable":true,"x":0.52,"y":0.76,'
            '"confidence":0.8,"reason":"20字内中文原因"}'
        )
        encoded = cv2.imencode(
            ".jpg", query_image, [cv2.IMWRITE_JPEG_QUALITY, 82]
        )[1].tobytes()
        started = time.monotonic()
        decision = query_json(self.args, prompt, encoded)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        if not decision.get("walkable", True):
            return {
                "ok": True,
                "walkable": False,
                "goal_publishing": False,
                "reason": str(decision.get("reason", ""))[:80],
                "elapsed_ms": round(elapsed_ms, 1),
            }

        x_norm = float(decision["x"])
        y_norm = float(decision["y"])
        if x_norm > 1.0 or y_norm > 1.0:
            x_norm /= image.shape[1]
            y_norm /= image.shape[0]
        x_norm = min(1.0, max(0.0, x_norm))
        y_norm = min(1.0, max(0.0, y_norm))
        u = min(image.shape[1] - 1, max(0, int(round(x_norm * (image.shape[1] - 1)))))
        v = min(image.shape[0] - 1, max(0, int(round(y_norm * (image.shape[0] - 1)))))

        camera_matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(camera_info.d, dtype=np.float64).reshape(-1, 1)
        normalized = cv2.fisheye.undistortPoints(
            np.asarray([[[float(u), float(v)]]], dtype=np.float64),
            camera_matrix,
            distortion,
        ).reshape(2)
        ray_camera = np.array(
            [normalized[0], normalized[1], 1.0], dtype=np.float64
        )
        ray_camera /= np.linalg.norm(ray_camera)

        camera_frame = header.frame_id or camera_info.header.frame_id
        query_time = Time.from_msg(header.stamp)
        transform, tf_latest = self.lookup_transform(camera_frame, query_time)
        rotation = quaternion_matrix(transform.transform.rotation)
        origin = np.array(
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
            float(np.dot(ground_normal, origin)) + ground_offset
        ) / denominator
        if ray_length < self.args.min_target_range:
            raise RuntimeError(
                "selected ground point is too close: %.3f m" % ray_length
            )
        if ray_length > self.args.max_range:
            raise RuntimeError(
                "ground intersection outside range: %.3f m" % ray_length
            )
        ground_intersection = origin + ray_length * ray_world
        target, target_debug = self.lidar_target_for_ray(
            origin,
            ray_world,
            ground_normal,
            ground_offset,
            ground_intersection,
            camera_frame,
            query_time,
            camera_info,
            np.asarray([u, v], dtype=np.float64),
        )
        camera_horizontal_range = math.hypot(
            float(target[0] - origin[0]),
            float(target[1] - origin[1]),
        )
        camera_slant_range = float(np.linalg.norm(target - origin))
        distance_transform, distance_tf_latest = self.lookup_transform(
            self.args.distance_frame, query_time
        )
        robot_origin = np.array(
            [
                distance_transform.transform.translation.x,
                distance_transform.transform.translation.y,
                distance_transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        robot_horizontal_range = math.hypot(
            float(target[0] - robot_origin[0]),
            float(target[1] - robot_origin[1]),
        )
        robot_slant_range = float(np.linalg.norm(target - robot_origin))
        if robot_horizontal_range > self.args.max_target_distance:
            raise RuntimeError(
                "selected target is too far from dog center: %.3f m > %.3f m"
                % (
                    robot_horizontal_range,
                    self.args.max_target_distance,
                )
            )
        return {
            "ok": True,
            "walkable": True,
            "goal_publishing": False,
            "pixel": [u, v],
            "pixel_normalized": [round(x_norm, 5), round(y_norm, 5)],
            "target_xyz": [round(float(value), 4) for value in target],
            "camera_origin": [round(float(value), 4) for value in origin],
            "robot_origin": [
                round(float(value), 4) for value in robot_origin
            ],
            "range_origin": self.args.distance_frame,
            "ray_camera": [round(float(value), 6) for value in ray_camera],
            "ray_world": [round(float(value), 6) for value in ray_world],
            "target_distance_m": round(robot_horizontal_range, 3),
            "target_slant_range_m": round(robot_slant_range, 3),
            "camera_target_distance_m": round(camera_horizontal_range, 3),
            "camera_target_slant_range_m": round(camera_slant_range, 3),
            "target_source": target_debug["source"],
            "ground_intersection_xyz": [
                round(float(value), 4) for value in ground_intersection
            ],
            "lidar_target": target_debug,
            "ground_plane": ground_debug,
            "locked": False,
            "confidence": max(
                0.0, min(1.0, float(decision.get("confidence", 0.0)))
            ),
            "reason": str(decision.get("reason", ""))[:80],
            "tf_latest": tf_latest,
            "distance_tf_latest": distance_tf_latest,
            "elapsed_ms": round(elapsed_ms, 1),
        }

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
        if target_camera[2] <= 0.05:
            projected["pixel"] = None
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
            projected["pixel"] = None
            return projected
        u_int = int(round(float(u)))
        v_int = int(round(float(v)))
        projected["pixel"] = [u_int, v_int]
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
        if not result.get("ok"):
            return

        annotated = image.copy()
        if result.get("pixel"):
            u, v = result["pixel"]
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
            label = "VLM FLOOR TARGET"
            detail = "px (%d,%d)  DOG %.2f m  %.0f%%" % (
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
            message = "WAYPOINT OUT OF VIEW  %.2f m" % distance
            if result.get("pixel") is None:
                message = "WAYPOINT BEHIND CAMERA  %.2f m" % distance
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
        origin = result["camera_origin"]
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
        default=5.5,
        help="maximum accepted horizontal target range from the distance frame",
    )
    parser.add_argument("--min-target-range", type=float, default=0.1)
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
    parser.add_argument("--api-timeout", type=float, default=60.0)
    parser.add_argument(
        "--env-file", default=os.path.expanduser("~/.env")
    )
    args = parser.parse_args()
    if (
        args.interval <= 0.0
        or args.max_range <= 0.1
        or args.max_target_distance <= args.min_target_range
        or args.min_target_range < 0.1
        or args.min_target_range >= args.max_range
    ):
        parser.error("interval or target range settings are invalid")
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

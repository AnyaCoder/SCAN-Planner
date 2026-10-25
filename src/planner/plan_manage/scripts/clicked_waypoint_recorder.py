#!/usr/bin/env python3
"""Record RViz Publish Point clicks as numbered SCAN waypoints."""

import argparse
import math
import os
import select
import sys
import termios
import tty
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_OUTPUT = os.path.join(os.getcwd(), "clicked_waypoints.yaml")


def format_float(value):
    text = f"{float(value):.6f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def atomic_write(path, content):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        stream.write(content)
    os.replace(temporary, path)


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    resolved: bool = False


class ClickedWaypointRecorder(Node):
    def __init__(
        self,
        topic,
        output,
        fixed_z,
        use_click_z,
        drive,
        goal_topic,
        odom_topic,
        arrival_threshold,
        arrival_speed,
    ):
        super().__init__("clicked_waypoint_recorder")
        self.output_path = os.path.abspath(output)
        self.fixed_z = fixed_z
        self.use_click_z = use_click_z
        self.drive = drive
        self.arrival_threshold = arrival_threshold
        self.arrival_speed = arrival_speed
        self.odom_xyz = None
        self.odom_speed = math.inf
        self.points = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, "/clicked_waypoints", qos)
        self.goal_pub = None
        if self.drive:
            self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.create_subscription(PointStamped, topic, self.on_point, 10)
        self.create_subscription(
            Odometry,
            odom_topic,
            self.on_odom,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(0.1, self.check_arrivals)
        self.publish_markers()
        self.get_logger().info(
            f"Click waypoints on {topic}; output={self.output_path}, fixed_z={fixed_z}"
        )
        self.get_logger().info(
            f"Arrival z: odom={odom_topic}, radius={arrival_threshold:.2f} m, "
            f"speed={arrival_speed:.2f} m/s"
        )
        if self.drive:
            self.get_logger().info(
                f"Drive-on-click enabled: publish goals to {goal_topic} (requires navi_mode=1)"
            )

    def on_point(self, msg):
        frame = msg.header.frame_id
        if frame and frame not in ("world", "map"):
            self.get_logger().warning(
                f"Ignoring point in frame '{frame}', expected world/map"
            )
            return
        z = msg.point.z
        if not self.use_click_z or not math.isfinite(z):
            z = self.fixed_z
        waypoint = Waypoint(float(msg.point.x), float(msg.point.y), float(z))
        if not all(
            math.isfinite(value) for value in (waypoint.x, waypoint.y, waypoint.z)
        ):
            return
        self.points.append(waypoint)
        self.publish_markers()
        if self.goal_pub is not None:
            goal = PoseStamped()
            goal.header.frame_id = "world"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = waypoint.x
            goal.pose.position.y = waypoint.y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
        self.get_logger().info(
            f"Waypoint {len(self.points)} pending z: "
            f"[{waypoint.x:.3f}, {waypoint.y:.3f}, {waypoint.z:.3f}]"
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.odom_xyz = (float(p.x), float(p.y), float(p.z))
        self.odom_speed = math.hypot(float(v.x), float(v.y))

    def check_arrivals(self):
        if self.odom_xyz is None:
            return
        x, y, z = self.odom_xyz
        resolved = False
        for index, waypoint in enumerate(self.points, 1):
            if waypoint.resolved:
                continue
            distance = math.hypot(waypoint.x - x, waypoint.y - y)
            if distance > self.arrival_threshold:
                continue
            if self.odom_speed > self.arrival_speed:
                continue
            waypoint.z = z
            waypoint.resolved = True
            resolved = True
            self.get_logger().info(
                f"Waypoint {index} z locked at arrival: "
                f"[{waypoint.x:.3f}, {waypoint.y:.3f}, {waypoint.z:.3f}] "
                f"distance={distance:.3f} speed={self.odom_speed:.3f}"
            )
        if resolved:
            self.publish_markers()

    def make_marker(self, marker_type, marker_id, namespace):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_markers(self):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)

        if self.points:
            line = self.make_marker(Marker.LINE_STRIP, 0, "clicked_waypoints")
            line.scale.x = 0.04
            line.color.r = 0.0
            line.color.g = 1.0
            line.color.b = 1.0
            line.color.a = 0.9
            for waypoint in self.points:
                point = Point()
                point.x, point.y, point.z = waypoint.x, waypoint.y, waypoint.z
                line.points.append(point)
            output.markers.append(line)

        for index, waypoint in enumerate(self.points, 1):
            sphere = self.make_marker(Marker.SPHERE, index, "clicked_waypoints")
            sphere.pose.position.x = waypoint.x
            sphere.pose.position.y = waypoint.y
            sphere.pose.position.z = waypoint.z
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.16
            if waypoint.resolved:
                sphere.color.r = 0.1
                sphere.color.g = 1.0
                sphere.color.b = 0.2
            else:
                sphere.color.r = 1.0
                sphere.color.g = 0.75
                sphere.color.b = 0.0
            sphere.color.a = 1.0
            output.markers.append(sphere)

            text = self.make_marker(Marker.TEXT_VIEW_FACING, 1000 + index, "clicked_labels")
            text.pose.position.x = waypoint.x
            text.pose.position.y = waypoint.y
            text.pose.position.z = waypoint.z + 0.28
            text.scale.z = 0.28
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = str(index)
            output.markers.append(text)

        self.marker_pub.publish(output)

    def undo(self):
        if not self.points:
            self.get_logger().warning("No waypoint to undo")
            return
        waypoint = self.points.pop()
        self.publish_markers()
        self.get_logger().info(
            f"Undo waypoint: [{waypoint.x:.3f}, {waypoint.y:.3f}, {waypoint.z:.3f}]"
        )

    def clear(self):
        self.points.clear()
        self.publish_markers()
        self.get_logger().info("Cleared all waypoints")

    def build_yaml(self):
        values = [
            format_float(value)
            for waypoint in self.points
            for value in (waypoint.x, waypoint.y, waypoint.z)
        ]
        return (
            "scan_planner_node:\n"
            "  ros__parameters:\n"
            "    fsm.navi_mode: 2\n"
            f"    fsm.waypoints: [{', '.join(values)}]\n"
        )

    def save(self):
        unresolved = [str(i) for i, point in enumerate(self.points, 1) if not point.resolved]
        if unresolved:
            self.get_logger().warning(
                "Unresolved arrival z for waypoint(s) "
                + ", ".join(unresolved)
                + "; using initial fixed/click z"
            )
        atomic_write(self.output_path, self.build_yaml())
        self.get_logger().info(f"Saved {len(self.points)} waypoint(s) to {self.output_path}")

    def list_points(self):
        for index, point in enumerate(self.points, 1):
            status = "locked" if point.resolved else "pending"
            print(f"{index}: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f}) {status}")

    def run_keyboard(self):
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard control requires a TTY")
        print("u: undo, c: clear, s: save, l: list, q: save+quit")
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not ready:
                    continue
                key = sys.stdin.read(1).lower()
                if key == "u":
                    self.undo()
                elif key == "c":
                    self.clear()
                elif key == "s":
                    self.save()
                elif key == "l":
                    self.list_points()
                elif key in ("q", "\x03"):
                    self.save()
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/clicked_point")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--z", type=float, default=0.4)
    parser.add_argument("--use-click-z", action="store_true")
    parser.add_argument(
        "--drive",
        action="store_true",
        help="also publish each click to /move_base_simple/goal (navi_mode=1)",
    )
    parser.add_argument("--goal-topic", default="/move_base_simple/goal")
    parser.add_argument("--odom-topic", default="/LIO/odom_vehicle")
    parser.add_argument("--arrival-threshold", type=float, default=0.30)
    parser.add_argument("--arrival-speed", type=float, default=0.15)
    parser.add_argument("--no-keyboard", action="store_true")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=sys.argv)
    recorder = ClickedWaypointRecorder(
        args.topic,
        args.output,
        args.z,
        args.use_click_z,
        args.drive,
        args.goal_topic,
        args.odom_topic,
        args.arrival_threshold,
        args.arrival_speed,
    )
    try:
        if args.no_keyboard:
            rclpy.spin(recorder)
        else:
            recorder.run_keyboard()
    except (KeyboardInterrupt, RuntimeError) as error:
        recorder.get_logger().error(str(error))
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

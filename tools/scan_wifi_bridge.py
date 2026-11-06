#!/usr/bin/env python3
"""Bridge the SCAN visualization graph across the Go2 Wi-Fi link over TCP.

The Go2 runs this file with --server and keeps its ROS graph on eth0.  The
workstation runs --client with its ROS graph on lo.  Only selected state topics
cross the Wi-Fi link; goals are sent back on the same TCP connection. The
server supports multiple concurrent clients with per-client topic filters.
"""

import argparse
import collections
import json
import socket
import struct
import threading
import time
import traceback
import zlib

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message

from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


MAGIC = b"SCWB"
VERSION = 3
FLAG_ZLIB = 1
HEADER = struct.Struct("!4sBBHHI")
MAX_PAYLOAD = 128 * 1024 * 1024
CLOCK_PROBE = "__scan_wifi_clock_probe__"
CLOCK_ACK = "__scan_wifi_clock_ack__"
CLOCK_APPLY = "__scan_wifi_clock_apply__"
ENABLE_COMPRESSION = False


class Topic:
    def __init__(
        self, name, type_name, message_cls, profile="reliable", interval=0.0, compress=False
    ):
        self.name = name
        self.type_name = type_name
        self.message_cls = message_cls
        self.profile = profile
        self.interval = interval
        self.compress = compress


UPLINK = [
    Topic("/robot_description", "std_msgs/msg/String", String, "latched"),
    Topic("/joint_states", "sensor_msgs/msg/JointState", JointState, "sensor", 0.05),
    Topic("/tf", "tf2_msgs/msg/TFMessage", TFMessage, "tf"),
    Topic("/tf_static", "tf2_msgs/msg/TFMessage", TFMessage, "latched"),
    Topic("/LIO/odom_vehicle", "nav_msgs/msg/Odometry", Odometry, "sensor"),
    Topic("/LIO/odom_imu", "nav_msgs/msg/Odometry", Odometry, "sensor"),
    Topic(
        "/LIO/relocalized/odom_vehicle",
        "nav_msgs/msg/Odometry",
        Odometry,
        "sensor",
    ),
    Topic("/LIO/clouds_lidar", "sensor_msgs/msg/PointCloud2", PointCloud2, "cloud", 0.20),
    Topic("/relocalization/map", "sensor_msgs/msg/PointCloud2", PointCloud2, "latched"),
    Topic("/relocalization/status", "std_msgs/msg/String", String, "latched"),
    Topic("/relocalization/fitness", "std_msgs/msg/Float32", Float32, "latched"),
    Topic("/grid_map/occupancy", "sensor_msgs/msg/PointCloud2", PointCloud2, "cloud", 0.50),
    Topic(
        "/grid_map/occupancy_inflate",
        "sensor_msgs/msg/PointCloud2",
        PointCloud2,
        "cloud",
        1.0,
        True,
    ),
    Topic("/grid_map/sliding_map_bbox", "visualization_msgs/msg/Marker", Marker, "reliable", 0.20),
    Topic("/goal_point", "visualization_msgs/msg/Marker", Marker, "reliable", 0.20),
    Topic("/self_inflation", "visualization_msgs/msg/Marker", Marker, "reliable", 0.20),
    Topic("/global_list", "visualization_msgs/msg/Marker", Marker, "reliable", 0.20),
    Topic("/optimal_list", "visualization_msgs/msg/Marker", Marker, "reliable", 0.20),
    Topic("/quad_0/path", "nav_msgs/msg/Path", Path, "reliable", 0.20),
]

DOWNLINK = [
    Topic("/move_base_simple/goal", "geometry_msgs/msg/PoseStamped", PoseStamped),
    Topic("/clicked_point", "geometry_msgs/msg/PointStamped", PointStamped),
    Topic(
        "/initialpose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        PoseWithCovarianceStamped,
    ),
]

TYPE_MAP = {topic.type_name: topic.message_cls for topic in UPLINK + DOWNLINK}


def make_qos(profile):
    if profile == "latched":
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    if profile == "tf":
        return QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
    if profile == "sensor":
        return QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
    if profile == "cloud":
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def output_qos(profile):
    if profile == "latched":
        return make_qos("latched")
    if profile == "cloud" or profile == "sensor":
        if profile == "cloud":
            return QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        return QoSProfile(depth=3, reliability=ReliabilityPolicy.RELIABLE)
    if profile == "reliable":
        return make_qos("latched")
    return make_qos(profile)


def encode_frame(topic, type_name, payload, compress=False):
    flags = 0
    if (ENABLE_COMPRESSION or compress) and len(payload) > 512:
        compressed = zlib.compress(payload, 1)
        if len(compressed) < len(payload):
            payload = compressed
            flags |= FLAG_ZLIB
    topic_bytes = topic.encode("utf-8")
    type_bytes = type_name.encode("utf-8")
    if len(topic_bytes) > 65535 or len(type_bytes) > 65535:
        raise ValueError("topic/type name too long")
    return (
        HEADER.pack(MAGIC, VERSION, flags, len(type_bytes), len(topic_bytes), len(payload))
        + type_bytes
        + topic_bytes
        + payload
    )


def recv_exact(sock, size, deadline=None):
    chunks = []
    remaining = size
    while remaining:
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError("bridge receive timed out")
        try:
            chunk = sock.recv(remaining)
        except socket.timeout:
            if deadline is not None:
                raise TimeoutError("bridge receive timed out")
            continue
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_frame(sock, deadline=None):
    header = recv_exact(sock, HEADER.size, deadline)
    if header is None:
        return None
    magic, version, flags, type_size, topic_size, payload_size = HEADER.unpack(header)
    if magic != MAGIC or version != VERSION or payload_size > MAX_PAYLOAD:
        raise ValueError("invalid bridge frame")
    type_bytes = recv_exact(sock, type_size, deadline)
    topic_bytes = recv_exact(sock, topic_size, deadline)
    payload = recv_exact(sock, payload_size, deadline)
    if type_bytes is None or topic_bytes is None or payload is None:
        return None
    if flags & FLAG_ZLIB:
        payload = zlib.decompress(payload)
    return topic_bytes.decode("utf-8"), type_bytes.decode("utf-8"), payload


class ClientSession:
    def __init__(self, conn, peer, subscriptions=None):
        self.conn = conn
        self.peer = peer
        self.subscriptions = subscriptions
        self.stop_event = threading.Event()
        self._condition = threading.Condition()
        self._pending = collections.OrderedDict()
        self._writer = None
        self.sent = 0

    def accepts(self, topic):
        return self.subscriptions is None or topic in self.subscriptions

    def offer(self, item):
        topic = item[0]
        if not self.accepts(topic):
            return
        with self._condition:
            if self.stop_event.is_set():
                return
            self._pending[topic] = item
            self._pending.move_to_end(topic)
            self._condition.notify()

    def pop(self, timeout):
        with self._condition:
            if not self._pending and not self.stop_event.is_set():
                self._condition.wait(timeout)
            if not self._pending:
                return None
            return self._pending.popitem(last=False)[1]

    def start(self):
        self._writer = threading.Thread(
            target=self._write_loop,
            name="bridge-client-%s" % (self.peer,),
            daemon=True,
        )
        self._writer.start()

    def _write_loop(self):
        while not self.stop_event.is_set() and rclpy.ok():
            item = self.pop(0.5)
            if item is None:
                continue
            try:
                self.conn.sendall(encode_frame(*item))
                self.sent += 1
            except (BrokenPipeError, ConnectionResetError, OSError):
                break
        self.close()

    def close(self):
        self.stop_event.set()
        with self._condition:
            self._pending.clear()
            self._condition.notify_all()


class FrameStore:
    SNAPSHOT_MAX_AGE = 2.0

    def __init__(self):
        self._lock = threading.Lock()
        self._latest = collections.OrderedDict()
        self._sessions = set()

    def wants(self, topic):
        with self._lock:
            return any(session.accepts(topic) for session in self._sessions)

    def add_client(self, session):
        with self._lock:
            self._sessions.add(session)

    def remove_client(self, session):
        with self._lock:
            self._sessions.discard(session)

    def snapshot(self, session):
        now = time.monotonic()
        with self._lock:
            items = list(self._latest.values())
        return [
            item
            for timestamp, item, persistent in items
            if session.accepts(item[0])
            and (persistent or now - timestamp <= self.SNAPSHOT_MAX_AGE)
        ]

    def push(self, item, persistent=False):
        topic = item[0]
        with self._lock:
            self._latest[topic] = (time.monotonic(), item, persistent)
            self._latest.move_to_end(topic)
            sessions = tuple(self._sessions)
        for session in sessions:
            session.offer(item)


def shift_stamp(stamp, delta_s):
    if stamp.sec == 0 and stamp.nanosec == 0:
        return
    stamp_ns = stamp.sec * 1000000000 + stamp.nanosec + int(round(delta_s * 1000000000))
    stamp.sec, stamp.nanosec = divmod(stamp_ns, 1000000000)


def shift_message_time(message, delta_s):
    if delta_s == 0.0:
        return
    if hasattr(message, "header"):
        shift_stamp(message.header.stamp, delta_s)
    if hasattr(message, "transforms"):
        for transform in message.transforms:
            shift_stamp(transform.header.stamp, delta_s)
    if hasattr(message, "poses"):
        for pose in message.poses:
            shift_stamp(pose.header.stamp, delta_s)


def clock_value(payload):
    return float(payload.decode("ascii"))


def clock_frame(topic, value):
    return encode_frame(topic, "clock", ("%.9f" % value).encode("ascii"))


def clock_ack_frame(value, topics):
    payload = json.dumps(
        {
            "time": value,
            "topics": None if topics is None else sorted(topics),
        },
        separators=(",", ":"),
    ).encode("ascii")
    return encode_frame(CLOCK_ACK, "clock", payload)


def clock_ack_value(payload):
    data = json.loads(payload.decode("ascii"))
    topics = data.get("topics")
    if topics is None:
        return float(data["time"]), None
    if not isinstance(topics, list):
        raise ValueError("clock topic filter must be a list")
    return float(data["time"]), {str(topic) for topic in topics}


def synchronize_clock(conn):
    probe_sent = time.time()
    conn.sendall(clock_frame(CLOCK_PROBE, probe_sent))
    frame = recv_frame(conn, time.monotonic() + 5.0)
    if frame is None:
        raise ConnectionError("Wi-Fi client disconnected during clock sync")
    topic, type_name, payload = frame
    if topic != CLOCK_ACK or type_name != "clock":
        raise ValueError("unexpected clock response %s (%s)" % (topic, type_name))
    probe_received, subscriptions = clock_ack_value(payload)
    response_received = time.time()
    offset_s = ((probe_sent - probe_received) + (response_received - probe_received)) / 2.0
    conn.sendall(clock_frame(CLOCK_APPLY, offset_s))
    return offset_s, subscriptions


class ServerNode(Node):
    def __init__(self):
        super().__init__("scan_wifi_state_server")
        self.store = FrameStore()
        self.last_emit = {}
        self._downlink_lock = threading.Lock()
        self.downlink = {topic.name: topic for topic in DOWNLINK}
        self.downlink_publishers = {
            topic.name: self.create_publisher(topic.message_cls, topic.name, make_qos("reliable"))
            for topic in DOWNLINK
        }

        for topic in UPLINK:
            group = ReentrantCallbackGroup()
            self.create_subscription(
                topic.message_cls,
                topic.name,
                lambda msg, spec=topic: self.on_uplink(spec, msg),
                make_qos(topic.profile),
                callback_group=group,
            )
        self.get_logger().info("SCAN Wi-Fi state server ready")

    def on_uplink(self, spec, msg):
        persistent = spec.profile == "latched"
        if not persistent and not self.store.wants(spec.name):
            return
        now = time.monotonic()
        if spec.interval > 0.0 and now - self.last_emit.get(spec.name, 0.0) < spec.interval:
            return
        self.last_emit[spec.name] = now
        try:
            item = (
                spec.name,
                spec.type_name,
                serialize_message(msg),
                spec.compress,
            )
            self.store.push(item, persistent)
        except Exception:
            self.get_logger().error("serialize failed for %s\n%s" % (spec.name, traceback.format_exc()))

    def publish_downlink(self, topic, type_name, payload):
        spec = self.downlink.get(topic)
        if spec is None or spec.type_name != type_name:
            self.get_logger().warning("drop unknown downlink %s (%s)" % (topic, type_name))
            return
        try:
            message = deserialize_message(payload, spec.message_cls)
            with self._downlink_lock:
                self.downlink_publishers[topic].publish(message)
            self.get_logger().info("downlink %s -> dog graph" % topic)
        except Exception:
            self.get_logger().error("downlink failed for %s\n%s" % (topic, traceback.format_exc()))


def serve_connection(conn, peer, node):
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    conn.settimeout(10.0)
    node.get_logger().info("Wi-Fi client connected: %s" % (peer,))
    try:
        offset_s, subscriptions = synchronize_clock(conn)
    except Exception:
        node.get_logger().warning("Wi-Fi clock sync failed: %s" % traceback.format_exc())
        conn.close()
        return

    if subscriptions is not None:
        known = {topic.name for topic in UPLINK}
        unknown = subscriptions - known
        if unknown:
            node.get_logger().warning(
                "client %s requested unknown topics: %s"
                % (peer, ", ".join(sorted(unknown)))
            )
        subscriptions &= known

    session = ClientSession(conn, peer, subscriptions)
    node.get_logger().info(
        "Wi-Fi clock offset for %s: Go2 - workstation = %.6f s, topics=%s"
        % (
            peer,
            offset_s,
            "all" if subscriptions is None else len(subscriptions),
        )
    )
    try:
        for item in node.store.snapshot(session):
            conn.sendall(encode_frame(*item))
            session.sent += 1
    except (BrokenPipeError, ConnectionResetError, OSError):
        conn.close()
        return

    node.store.add_client(session)
    session.start()
    try:
        conn.settimeout(1.0)
        while not session.stop_event.is_set() and rclpy.ok():
            try:
                frame = recv_frame(conn, time.monotonic() + 1.0)
            except TimeoutError:
                continue
            if frame is None:
                break
            node.publish_downlink(*frame)
    except (ConnectionResetError, OSError, ValueError):
        if rclpy.ok():
            node.get_logger().warning(
                "Wi-Fi client %s read failed: %s"
                % (peer, traceback.format_exc())
            )
    finally:
        node.store.remove_client(session)
        session.close()
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        conn.close()
        node.get_logger().info(
            "Wi-Fi client %s disconnected (sent=%d)" % (peer, session.sent)
        )


def run_server(args):
    rclpy.init()
    node = ServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.bind, args.port))
    server.listen(8)
    server.settimeout(1.0)
    node.get_logger().info(
        "listening on %s:%d (compression=%s, multi-client)"
        % (args.bind, args.port, "on" if ENABLE_COMPRESSION else "off")
    )
    client_threads = set()
    try:
        while rclpy.ok():
            try:
                conn, peer = server.accept()
            except socket.timeout:
                continue
            client_threads = {thread for thread in client_threads if thread.is_alive()}
            thread = threading.Thread(
                target=serve_connection,
                args=(conn, peer, node),
                name="bridge-client-%s" % (peer,),
                daemon=True,
            )
            client_threads.add(thread)
            thread.start()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        for thread in client_threads:
            thread.join(timeout=1.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class ClientNode(Node):
    def __init__(self):
        super().__init__("scan_wifi_state_client")
        self._connection = None
        self._connection_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._clock_offset = 0.0
        self._topic_publishers = {}
        for topic in DOWNLINK:
            group = ReentrantCallbackGroup()
            self.create_subscription(
                topic.message_cls,
                topic.name,
                lambda msg, spec=topic: self.on_downlink(spec, msg),
                make_qos("reliable"),
                callback_group=group,
            )
        self.get_logger().info("SCAN Wi-Fi state client ready")

    def attach(self, conn):
        with self._connection_lock:
            self._connection = conn

    def detach(self):
        with self._connection_lock:
            self._connection = None

    def set_clock_offset(self, offset_s):
        self._clock_offset = offset_s

    def on_downlink(self, spec, msg):
        shift_message_time(msg, self._clock_offset)
        frame = encode_frame(spec.name, spec.type_name, serialize_message(msg))
        with self._connection_lock:
            conn = self._connection
            if conn is None:
                self.get_logger().warning("drop %s: Go2 Wi-Fi bridge disconnected" % spec.name)
                return
            try:
                with self._send_lock:
                    conn.sendall(frame)
                self.get_logger().info("downlink %s -> Go2" % spec.name)
            except OSError as exc:
                self.get_logger().error("send %s failed: %s" % (spec.name, exc))

    def publish_uplink(self, topic, type_name, payload):
        message_cls = TYPE_MAP.get(type_name)
        if message_cls is None:
            self.get_logger().warning("unknown message type %s" % type_name)
            return
        publisher = self._topic_publishers.get(topic)
        if publisher is None:
            spec = next((item for item in UPLINK if item.name == topic), None)
            profile = spec.profile if spec is not None else "reliable"
            publisher = self.create_publisher(message_cls, topic, output_qos(profile))
            self._topic_publishers[topic] = publisher
        message = deserialize_message(payload, message_cls)
        shift_message_time(message, -self._clock_offset)
        publisher.publish(message)


def parse_topic_filter(value):
    if not value or value == "all":
        return None
    if value == "none":
        return set()
    return {item.strip() for item in value.split(",") if item.strip()}


def run_client(args):
    rclpy.init()
    node = ClientNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    count = 0
    last_report = time.monotonic()
    requested_topics = parse_topic_filter(args.topics)
    if requested_topics is not None:
        unknown = requested_topics - {topic.name for topic in UPLINK}
        if unknown:
            node.get_logger().warning(
                "ignoring unknown uplink topics: %s"
                % ", ".join(sorted(unknown))
            )
        requested_topics &= {topic.name for topic in UPLINK}
    node.get_logger().info(
        "requested uplink topics: %s"
        % ("all" if requested_topics is None else len(requested_topics))
    )
    try:
        while rclpy.ok():
            try:
                conn = socket.create_connection((args.host, args.port), timeout=5.0)
            except OSError as exc:
                node.get_logger().warning(
                    "waiting for Go2 Wi-Fi bridge %s:%d (%s)" % (args.host, args.port, exc)
                )
                time.sleep(2.0)
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            conn.settimeout(10.0)
            node.get_logger().info("connected to Go2 Wi-Fi bridge %s:%d" % (args.host, args.port))
            try:
                while rclpy.ok():
                    frame = recv_frame(conn)
                    if frame is None:
                        break
                    topic, type_name, payload = frame
                    if topic == CLOCK_PROBE and type_name == "clock":
                        conn.sendall(
                            clock_ack_frame(time.time(), requested_topics)
                        )
                        continue
                    if topic == CLOCK_APPLY and type_name == "clock":
                        offset_s = clock_value(payload)
                        node.set_clock_offset(offset_s)
                        node.attach(conn)
                        node.get_logger().info(
                            "Wi-Fi clock offset: Go2 - workstation = %.6f s" % offset_s
                        )
                        continue
                    node.publish_uplink(topic, type_name, payload)
                    count += 1
                    now = time.monotonic()
                    if now - last_report >= 10.0:
                        node.get_logger().info("uplink frames=%d" % count)
                        last_report = now
            except (ConnectionResetError, OSError, ValueError) as exc:
                if rclpy.ok():
                    node.get_logger().warning("Go2 Wi-Fi bridge disconnected: %s" % exc)
            finally:
                node.detach()
                conn.close()
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    global ENABLE_COMPRESSION
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--server", action="store_true")
    mode.add_argument("--client", action="store_true")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--host", default="192.168.76.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument(
        "--topics",
        default="",
        help=(
            "client uplink topic filter: comma-separated names, "
            "'all' (default), or 'none'"
        ),
    )
    parser.add_argument("--compress", action="store_true")
    args = parser.parse_args()
    ENABLE_COMPRESSION = args.compress
    if args.server:
        run_server(args)
    else:
        run_client(args)


if __name__ == "__main__":
    main()

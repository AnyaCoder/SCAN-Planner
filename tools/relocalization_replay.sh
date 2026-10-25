#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=${SCAN_LOCAL_WS:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
GO2_HOST=${GO2_HOST:-go2-wifi}
GO2_WIFI_HOST=${GO2_WIFI_HOST:-192.168.76.1}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-/home/unitree/SCAN-Planner}
REMOTE_MAP=$REMOTE_ROOT/maps/relocalization_map.pcd
REMOTE_WAYPOINTS=$REMOTE_ROOT/maps/clicked_waypoints.yaml
REMOTE_LOG_DIR=/tmp/relocalization_logs
LOCAL_LOG_DIR=/tmp/relocalization_logs_local
LOCAL_DDS_CONFIG=${SCAN_LOCAL_DDS_CONFIG:-$HOME/.config/ros2/cyclonedds.xml}
REMOTE_DDS_CONFIG=/home/unitree/.config/ros2/cyclonedds.xml
RELOC_TIMEOUT=${SCAN_RELOCALIZATION_TIMEOUT:-180}
NAVI_MODE=${SCAN_NAVI_MODE:-2}
RVIZ_CONFIG=$WS/install/map_relocalizer/share/map_relocalizer/rviz/relocalization.rviz

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<EOF
用法:
  $0 start [bundle_dir|map_world.pcd] [waypoints.yaml]
  $0 replay
  $0 stop
  $0 status

start  清理旧服务，启动完整重定位链，等待 2D Pose Estimate 和 ICP 收敛，
       然后启动速度桥与 SCAN 路径点回放。默认使用最新 bundle。
replay 重定位已收敛时，只重启 SCAN 的 navi_mode=$NAVI_MODE 回放。
stop   停止完整重定位链和 SCAN。
status 查看进程、重定位状态和当前回放日志。

环境变量:
  SCAN_NAVI_MODE=2
  SCAN_RELOCALIZATION_TIMEOUT=180
  GO2_HOST=go2-wifi
  GO2_WIFI_HOST=192.168.76.1
EOF
}

source_local() {
  set +u
  source /opt/ros/foxy/setup.bash
  # shellcheck disable=SC1090
  source "$WS/install/local_setup.bash"
  set -u
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$LOCAL_DDS_CONFIG
  export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
}

check_prereqs() {
  [[ -n "${DISPLAY:-}" ]] || die "当前终端没有 DISPLAY"
  [[ -f "$LOCAL_DDS_CONFIG" ]] || die "缺少工作站 DDS 配置: $LOCAL_DDS_CONFIG"
  [[ -x "$WS/install/map_relocalizer/lib/map_relocalizer/map_relocalizer_node" ]] || \
    die "map_relocalizer 未编译"
  [[ -f "$RVIZ_CONFIG" ]] || die "缺少重定位 RViz 配置: $RVIZ_CONFIG"
  ssh -o ConnectTimeout=5 "$GO2_HOST" true || die "无法 SSH 连接 $GO2_HOST"
  grep -Fq -- 'name="lo"' "$LOCAL_DDS_CONFIG" || die "工作站 DDS 未绑定 lo"
  # shellcheck disable=SC2029
  ssh "$GO2_HOST" "grep -Fq 'name=\"eth0\"' '$REMOTE_DDS_CONFIG'" || \
    die "Go2 DDS 未绑定 eth0"
  ping -c 1 -W 2 "$GO2_WIFI_HOST" >/dev/null 2>&1 || die "Go2 Wi-Fi 不通"
}

resolve_bundle() {
  local input=${1:-} waypoint_arg=${2:-}
  if [[ -z "$input" ]]; then
    input=$(find "$WS/maps/bundles" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -1)
    [[ -n "$input" ]] || die "没有可用 bundle"
  fi
  if [[ -d "$input" ]]; then
    BUNDLE_DIR=$input
    MAP_FILE=$input/map_world.pcd
    WAYPOINTS_FILE=$input/clicked_waypoints.yaml
  elif [[ -f "$input" ]]; then
    MAP_FILE=$input
    BUNDLE_DIR=$(dirname "$input")
    WAYPOINTS_FILE=${waypoint_arg:-$BUNDLE_DIR/clicked_waypoints.yaml}
    [[ -f "$WAYPOINTS_FILE" ]] || WAYPOINTS_FILE=$WS/maps/clicked_waypoints.yaml
  else
    die "找不到 bundle 或 PCD: $input"
  fi
  [[ -f "$MAP_FILE" ]] || die "找不到 prior map: $MAP_FILE"
  [[ -f "$WAYPOINTS_FILE" ]] || die "找不到路径点: $WAYPOINTS_FILE"
}

stop_local_pgids() {
  local label=$1 pattern=$2
  local pgids=() ownpg
  ownpg=$(ps -o pgid= -p $$ | tr -d ' ')
  mapfile -t pgids < <(
    ps -eo pid=,pgid=,args= | PATTERN="$pattern" awk -v own="$$" -v ownpg="$ownpg" \
      '$1 != own && $2 != ownpg && $0 ~ ENVIRON["PATTERN"] {print $2}' | sort -u
  )
  ((${#pgids[@]})) || return 0
  printf '停止 %s: PGID %s\n' "$label" "${pgids[*]}"
  local pgid
  for pgid in "${pgids[@]}"; do kill -TERM -- "-$pgid" 2>/dev/null || true; done
  sleep 1
  for pgid in "${pgids[@]}"; do kill -KILL -- "-$pgid" 2>/dev/null || true; done
}

stop_chain() {
  printf '停止旧链路...\n'
  "$WS/tools/stop_scan_stack.sh" || true
  ssh "$GO2_HOST" 'bash -s' <<'REMOTE' || true
set -u
pids=$(pgrep -f '/lib/map_relocalizer/map_relocalizer_node' || true)
if [ -n "$pids" ]; then
  echo "停止 map_relocalizer: $pids"
  kill -TERM $pids 2>/dev/null || true
fi
REMOTE
  stop_local_pgids "Relocalization RViz" 'rviz2 .*relocalization\.rviz'
  stop_local_pgids "Wi-Fi TCP client" 'scan_wifi_bridge\.py --client'
}

start_remote_chain() {
  scp -q "$MAP_FILE" "$GO2_HOST:$REMOTE_MAP"
  scp -q "$WAYPOINTS_FILE" "$GO2_HOST:$REMOTE_WAYPOINTS"
  scp -q "$SCRIPT_DIR/scan_wifi_bridge.py" "$GO2_HOST:$REMOTE_ROOT/tools/scan_wifi_bridge.py"
  # shellcheck disable=SC2087
  ssh "$GO2_HOST" 'bash -s' <<REMOTE
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/fastlio_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/local_setup.bash
source /home/unitree/SCAN-Planner/install/local_setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$REMOTE_DDS_CONFIG
export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}
mkdir -p "$REMOTE_LOG_DIR"
setsid -f bash -c "cd /unitree/module/graph_pid_ws && exec ros2 launch livox_ros_driver2 msg_MID360_launch.py" >"$REMOTE_LOG_DIR/livox.log" 2>&1 </dev/null
setsid -f bash -c "cd $REMOTE_ROOT && exec ros2 launch fast_lio mapping.launch.py rviz:=false config_path:=$REMOTE_ROOT/src/planner/lio_bridge/config config_file:=mid360_fastlio.yaml tf_topic:=/fastlio/tf" >"$REMOTE_LOG_DIR/fastlio.log" 2>&1 </dev/null
setsid -f bash -c "cd $REMOTE_ROOT && exec ros2 launch lio_bridge fastlio_bridge.launch.py output_frame_id:=fastlio_world" >"$REMOTE_LOG_DIR/lio_bridge.log" 2>&1 </dev/null
setsid -f bash -c "cd $REMOTE_ROOT && exec ros2 launch map_relocalizer map_relocalizer.launch.py prior_map_file:=$REMOTE_MAP rviz:=false" >"$REMOTE_LOG_DIR/map_relocalizer.log" 2>&1 </dev/null
setsid -f bash -c "cd $REMOTE_ROOT && exec python3 tools/scan_wifi_bridge.py --server --bind 0.0.0.0 --port 9000" >"$REMOTE_LOG_DIR/wifi_bridge.log" 2>&1 </dev/null
REMOTE
}

start_local_bridge() {
  mkdir -p "$LOCAL_LOG_DIR"
  setsid -f bash -c "cd '$WS' && exec python3 tools/scan_wifi_bridge.py --client --host '$GO2_WIFI_HOST' --port 9000" \
    >"$LOCAL_LOG_DIR/wifi_bridge.log" 2>&1 </dev/null
}

wait_local_topic() {
  local topic=$1 deadline=$((SECONDS + ${2:-30}))
  while ((SECONDS < deadline)); do
    if timeout 4 ros2 topic list --no-daemon --spin-time 1 2>/dev/null | grep -Fxq -- "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

check_relocalization_converged() {
  timeout 6 python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

class Reader(Node):
    def __init__(self):
        super().__init__('wait_relocalization_status')
        self.status = ''
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, '/relocalization/status', self.cb, qos)

    def cb(self, msg):
        self.status = msg.data

rclpy.init()
node = Reader()
while rclpy.ok() and not node.status:
    rclpy.spin_once(node, timeout_sec=0.2)
ok = node.status.startswith('converged')
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
PY
}

wait_for_converged() {
  local deadline=$((SECONDS + RELOC_TIMEOUT))
  if check_relocalization_converged; then
    return 0
  fi
  printf '请在重定位 RViz 中使用 2D Pose Estimate 给初始位姿...\n'
  while ((SECONDS < deadline)); do
    if check_relocalization_converged; then
      return 0
    fi
    sleep 2
  done
  return 1
}

start_replay_nodes() {
  scp -q "$SCRIPT_DIR/publish_unitree_joint_states.py" \
    "$GO2_HOST:$REMOTE_ROOT/tools/publish_unitree_joint_states.py"
  # shellcheck disable=SC2087
  ssh "$GO2_HOST" 'bash -s' <<REMOTE
set +u
source /opt/ros/foxy/setup.bash
source /unitree/module/graph_pid_ws/install/local_setup.bash
source /home/unitree/fastlio_ws/install/local_setup.bash
source /home/unitree/autonomy_stack_go2/install/local_setup.bash
source /home/unitree/SCAN-Planner/install/local_setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$REMOTE_DDS_CONFIG
export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}
mkdir -p "$REMOTE_LOG_DIR"
if ! pgrep -f '[p]ublish_unitree_joint_states.py' >/dev/null 2>&1; then
  setsid -f bash -c "cd $REMOTE_ROOT && exec python3 tools/publish_unitree_joint_states.py" >"$REMOTE_LOG_DIR/joint_states.log" 2>&1 </dev/null
fi
if ! pgrep -f '/lib/cmd_vel_sport_bridge/cmd_vel_sport_bridge' >/dev/null 2>&1; then
  setsid -f bash -c "cd $REMOTE_ROOT && exec ros2 launch cmd_vel_sport_bridge cmd_vel_sport_bridge.launch.py" >"$REMOTE_LOG_DIR/cmd_vel_sport.log" 2>&1 </dev/null
fi
sleep 2
setsid -f bash -c "cd $REMOTE_ROOT && exec ros2 launch scan_planner run.launch.py is_real_world:=true use_relocalized_odom:=true navi_mode:=$NAVI_MODE keypoints_file:=$REMOTE_WAYPOINTS sensor_type:=lidar controller_mode:=closed_loop publish_robot_state:=true" >"$REMOTE_LOG_DIR/scan.log" 2>&1 </dev/null
REMOTE
}

start_local_rviz() {
  setsid -f bash -c "cd '$WS' && exec ros2 launch scan_planner rviz.launch.py rviz_config:='$RVIZ_CONFIG'" \
    >"$LOCAL_LOG_DIR/relocalization_rviz.log" 2>&1 </dev/null
}

start_chain() {
  resolve_bundle "${1:-}" "${2:-}"
  check_prereqs
  source_local
  stop_chain
  printf 'Bundle: %s\n' "$BUNDLE_DIR"
  start_remote_chain
  start_local_bridge
  wait_local_topic /relocalization/map 40 || die "工作站未收到 /relocalization/map"
  wait_local_topic /relocalization/status 30 || die "工作站未收到 /relocalization/status"
  start_local_rviz
  wait_for_converged || die "ICP 未在 ${RELOC_TIMEOUT}s 内收敛"
  printf 'ICP 已收敛。3 秒后启动速度桥和路径点回放...\n'
  sleep 3
  start_replay_nodes
  sleep 4
  printf '\n重定位回放已启动。\n'
  printf '  Go2 日志: ssh %s tail -f %s/{map_relocalizer,scan,cmd_vel_sport}.log\n' \
    "$GO2_HOST" "$REMOTE_LOG_DIR"
  printf '  停止: %s stop\n' "$0"
}

replay_only() {
  check_prereqs
  source_local
  wait_for_converged || die "重定位尚未收敛，不能重放"
  ssh "$GO2_HOST" 'bash -s' <<'REMOTE'
pids=$(pgrep -f '/lib/scan_planner/scan_planner_node|/lib/scan_planner/closed_loop_controller' || true)
if [ -n "$pids" ]; then kill -TERM $pids 2>/dev/null || true; fi
sleep 2
REMOTE
  start_replay_nodes
  sleep 4
  printf '已重新启动 SCAN 回放。\n'
}

status_chain() {
  source_local
  printf '=== local ===\n'
  pgrep -af 'rviz2 .*relocalization|scan_wifi_bridge.py --client' || true
  printf '=== remote ===\n'
  ssh "$GO2_HOST" "ps -eo pid,args | grep -E 'livox_ros_driver2_node|fastlio_mapping|lio_bridge_node|map_relocalizer_node|cmd_vel_sport_bridge|scan_planner_node|closed_loop_controller|scan_wifi_bridge.py --server' | grep -v grep || true" || true
  printf '=== relocalization ===\n'
  timeout 6 python3 - <<'PY' || true
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
class R(Node):
    def __init__(self):
        super().__init__('relocalization_status_show')
        self.s = None
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/relocalization/status', self.cb, q)
    def cb(self, msg):
        self.s = msg.data
rclpy.init(); n = R()
import time
end = time.time() + 4
while rclpy.ok() and n.s is None and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
print(n.s or 'no status')
rclpy.shutdown()
PY
  printf '=== scan tail ===\n'
  # shellcheck disable=SC2029
  ssh "$GO2_HOST" "tail -n 12 $REMOTE_LOG_DIR/scan.log 2>/dev/null || true" || true
}

cmd=${1:-start}
case "$cmd" in
  start) shift; start_chain "${1:-}" "${2:-}" ;;
  replay) replay_only ;;
  stop) stop_chain ;;
  status) status_chain ;;
  -h|--help|help) usage ;;
  *)
    if [[ -e "$cmd" ]]; then start_chain "$cmd" "${2:-}"; else usage; exit 2; fi
    ;;
esac

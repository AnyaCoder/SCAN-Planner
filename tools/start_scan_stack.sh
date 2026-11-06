#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
STOP_SCRIPT="$SCRIPT_DIR/stop_scan_stack.sh"
# shellcheck source=tools/stack_lib.sh
source "$SCRIPT_DIR/stack_lib.sh"

GO2_HOST=${GO2_HOST:-go2-wifi}
REMOTE_HOME=${SCAN_REMOTE_HOME:-/home/unitree}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-$REMOTE_HOME/SCAN-Planner}
REMOTE_GRAPH_WS=${SCAN_REMOTE_GRAPH_WS:-/unitree/module/graph_pid_ws}
REMOTE_FASTLIO_WS=${SCAN_REMOTE_FASTLIO_WS:-$REMOTE_HOME/fastlio_ws}
REMOTE_AUTONOMY_WS=${SCAN_REMOTE_AUTONOMY_WS:-$REMOTE_HOME/autonomy_stack_go2}
REMOTE_LOG_DIR=${SCAN_REMOTE_LOG_DIR:-/tmp/scan_planner_logs}
LOCAL_LOG_DIR=${SCAN_LOCAL_LOG_DIR:-/tmp/scan_planner_local_logs}
LOCAL_WS=${SCAN_LOCAL_WS:-$HOME/SCAN-Planner/foxy_ws}
ROS_DOMAIN_ID=${SCAN_ROS_DOMAIN_ID:-0}
NAVI_MODE=${SCAN_NAVI_MODE:-1}
KEYPOINTS_FILE=${SCAN_KEYPOINTS_FILE:-}
CLICKED_WAYPOINTS_ENABLE=${SCAN_CLICKED_WAYPOINTS_ENABLE:-auto}
CLICKED_WAYPOINTS_FILE=${SCAN_CLICKED_WAYPOINTS_FILE:-$LOCAL_WS/maps/clicked_waypoints.yaml}
CLICKED_WAYPOINTS_Z=${SCAN_CLICKED_WAYPOINTS_Z:-0.4}
CLICKED_WAYPOINTS_DRIVE=${SCAN_CLICKED_WAYPOINTS_DRIVE:-1}
GO2_WIFI_HOST=${GO2_WIFI_HOST:-192.168.76.1}
LOCAL_DDS_CONFIG=${SCAN_LOCAL_DDS_CONFIG:-$HOME/.config/ros2/cyclonedds.xml}
REMOTE_DDS_CONFIG=${SCAN_REMOTE_DDS_CONFIG:-$REMOTE_HOME/.config/ros2/cyclonedds.xml}
LOCAL_PYTHON_EXECUTABLE=${SCAN_PYTHON:-/usr/bin/python3}
REMOTE_PYTHON_EXECUTABLE=${SCAN_REMOTE_PYTHON:-/usr/bin/python3}

export ROS_DOMAIN_ID

CLEANUP_ON_FAILURE=0

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

front_camera_defaults || die "SCAN_CAMERA_ENABLE 只能是 1/0: $CAMERA_ENABLE"

cleanup_on_exit() {
  local rc=$?
  if (( rc != 0 && CLEANUP_ON_FAILURE == 1 )); then
    printf '\n启动未完成，正在清理已拉起的服务...\n' >&2
    "$STOP_SCRIPT" || true
  fi
  exit "$rc"
}
trap cleanup_on_exit EXIT

remote_env() {
  cat <<EOF
set -e
source /opt/ros/foxy/setup.bash
source "$REMOTE_GRAPH_WS/install/local_setup.bash"
source "$REMOTE_FASTLIO_WS/install/local_setup.bash"
source "$REMOTE_AUTONOMY_WS/install/local_setup.bash"
source "$REMOTE_ROOT/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export CYCLONEDDS_URI=file://$REMOTE_DDS_CONFIG
export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}
EOF
}

remote_run() {
  local body=$1
  local script
  script=$(printf '%s\n%s\n' "$(remote_env)" "$body")
  ssh "$GO2_HOST" bash -s <<<"$script"
}

start_remote() {
  local name=$1
  local workdir=$2
  local command=$3
  local log="$REMOTE_LOG_DIR/$name.log"
  local body
  body=$(cat <<EOF
mkdir -p $(printf '%q' "$REMOTE_LOG_DIR")
setsid -f bash -c $(printf '%q' "cd $(printf '%q' "$workdir") && exec $command") \
  >$(printf '%q' "$log") 2>&1 </dev/null
printf 'started %s\n' $(printf '%q' "$name")
printf 'log %s\n' $(printf '%q' "$log")
EOF
)
  remote_run "$body"
}

start_local() {
  local name=$1
  local command=$2
  local log="$LOCAL_LOG_DIR/$name.log"
  mkdir -p "$LOCAL_LOG_DIR"
  setsid -f bash -c "cd $(printf '%q' "$LOCAL_WS") && exec $command" \
    >"$log" 2>&1 </dev/null
  printf 'started %s\nlog %s\n' "$name" "$log"
}

open_local_terminal() {
  local title=$1
  local body=$2
  local error_file
  error_file=$(mktemp)
  if command -v gnome-terminal >/dev/null 2>&1; then
    # The persistent GNOME Terminal server can keep a stale screen object.
    # A private server is slower to start but reliable after desktop restarts.
    # Run it detached because --disable-factory waits for the terminal to close.
    setsid -f gnome-terminal --disable-factory --title="$title" \
      --working-directory="$LOCAL_WS" -- bash -c "$body" \
      >/dev/null 2>"$error_file" || true
    sleep 0.5
    if ! grep -Fq 'Error creating terminal' "$error_file"; then
      rm -f "$error_file"
      return 0
    fi
    cat "$error_file" >&2 || true
  fi
  if command -v x-terminal-emulator >/dev/null 2>&1; then
    setsid -f bash -c \
      "cd $(printf '%q' "$LOCAL_WS") && exec x-terminal-emulator -e bash -c $(printf '%q' "$body")" \
      >/dev/null 2>"$error_file" || true
    sleep 0.5
    if ! grep -Fq 'Error creating terminal' "$error_file"; then
      rm -f "$error_file"
      return 0
    fi
    cat "$error_file" >&2 || true
  fi
  rm -f "$error_file"
  return 1
}

wait_for_remote_topic() {
  local topic=$1
  local timeout_s=$2
  local body
  body=$(cat <<EOF
deadline=\$((SECONDS + $timeout_s))
while (( SECONDS < deadline )); do
  if timeout 4 ros2 topic list --no-daemon --spin-time 1 2>/dev/null | grep -Fxq -- $(printf '%q' "$topic"); then
    exit 0
  fi
  sleep 1
done
exit 1
EOF
)
  remote_run "$body"
}

wait_for_remote_node() {
  local node=$1
  local timeout_s=$2
  local body
  body=$(cat <<EOF
deadline=\$((SECONDS + $timeout_s))
while (( SECONDS < deadline )); do
  if timeout 4 ros2 node list --no-daemon --spin-time 1 2>/dev/null | grep -Fxq -- $(printf '%q' "$node"); then
    exit 0
  fi
  sleep 1
done
exit 1
EOF
)
  remote_run "$body"
}

wait_for_remote_log() {
  local log=$1
  local needle=$2
  local timeout_s=$3
  local body
  body=$(cat <<EOF
deadline=\$((SECONDS + $timeout_s))
while (( SECONDS < deadline )); do
  if [[ -f $(printf '%q' "$log") ]] && grep -Fq -- $(printf '%q' "$needle") $(printf '%q' "$log"); then
    exit 0
  fi
  sleep 1
done
exit 1
EOF
)
  remote_run "$body"
}

wait_for_local_log() {
  local log=$1
  local needle=$2
  local timeout_s=$3
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if [[ -f "$log" ]] && grep -Fq -- "$needle" "$log"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

show_remote_log() {
  local log=$1
  printf '\n--- %s ---\n' "$log" >&2
  remote_run "tail -n 80 $(printf '%q' "$log") 2>/dev/null || true" >&2 || true
}

remote_stack_running() {
  local body
  body=$(cat <<'EOF'
if pgrep -f 'native_wifi_relay_cpp\.new --(source|wifi)' >/dev/null 2>&1; then
  exit 0
fi
if timeout 4 ros2 node list --no-daemon 2>/dev/null | grep -FxEq -- '/(scan_planner_node|lio_bridge|fastlio_mapping|livox_lidar_publisher)'; then
  exit 0
fi
exit 1
EOF
)
  remote_run "$body" >/dev/null 2>&1
}

local_rviz_running() {
  pgrep -f 'rviz2 .*default.*\.rviz|ros2 launch scan_planner rviz\.launch\.py|scan_wifi_bridge\.py --client|clicked_waypoint_recorder\.py' >/dev/null 2>&1 || \
    front_camera_relay_running || front_camera_decoder_running
}

clicked_waypoints_enabled() {
  case "$CLICKED_WAYPOINTS_ENABLE" in
    1|true|on) return 0 ;;
    0|false|off) return 1 ;;
    auto) [[ "$NAVI_MODE" == "1" ]] ;;
    *)
      die "SCAN_CLICKED_WAYPOINTS_ENABLE 只能是 auto/1/0: $CLICKED_WAYPOINTS_ENABLE"
      ;;
  esac
}

check_dds_config() {
  local route local_ip local_iface local_cfg
  route=$(ip -4 route get "$GO2_WIFI_HOST" 2>/dev/null || true)
  local_ip=$(awk '{for (i = 1; i <= NF; ++i) if ($i == "src") {print $(i + 1); exit}}' <<<"$route")
  local_iface=$(awk '{for (i = 1; i <= NF; ++i) if ($i == "dev") {print $(i + 1); exit}}' <<<"$route")

  [[ -n "$local_ip" && -n "$local_iface" ]] || \
    die "找不到到 $GO2_WIFI_HOST 的路由，请先连接 Go2 WiFi"

  local_cfg=$LOCAL_DDS_CONFIG
  [[ -f "$local_cfg" ]] || die "缺少工作站 DDS 配置: $local_cfg"
  grep -Fq -- 'name="lo"' "$local_cfg" || \
    die "工作站 DDS 配置未绑定 lo: $local_cfg"
  if grep -Fq -- 'name="wlan0"' "$local_cfg" || grep -Fq -- 'name="wl' "$local_cfg"; then
    die "工作站 DDS 不得绑定 Wi-Fi，只应绑定 lo: $local_cfg"
  fi

  # shellcheck disable=SC2029
  if ! ssh "$GO2_HOST" \
    "test -f '$REMOTE_DDS_CONFIG' && \
     grep -Fq -- 'name=\"eth0\"' '$REMOTE_DDS_CONFIG' && \
     ! grep -Fq -- 'name=\"wlan0\"' '$REMOTE_DDS_CONFIG'"; then
    die "Go2 DDS 配置必须只绑定 eth0: $REMOTE_DDS_CONFIG"
  fi

  printf 'DDS: Go2 eth0 <-> 工作站 lo；TCP 桥经 Wi-Fi %s (%s) -> %s:9000\n' \
    "$local_iface" "$local_ip" "$GO2_WIFI_HOST"
  if command -v iw >/dev/null 2>&1 && iw dev "$local_iface" get power_save 2>/dev/null | grep -q on; then
    printf '警告: %s 开启了 Wi-Fi 省电，TCP 延迟会抖动。建议:\n' "$local_iface" >&2
    printf '  nmcli connection modify <Go2连接名> 802-11-wireless.powersave 2 && nmcli connection up <Go2连接名>\n' >&2
  fi
}

check_local_workspace() {
  [[ -n "${DISPLAY:-}" ]] || die "当前终端没有 DISPLAY，无法打开 RViz"
  [[ -f /opt/ros/foxy/setup.bash ]] || die "缺少 /opt/ros/foxy/setup.bash"
  [[ -f "$LOCAL_WS/install/local_setup.bash" ]] || \
    die "缺少 $LOCAL_WS/install/local_setup.bash，请先编译 workstation 工作空间"

  # ROS Foxy setup scripts reference optional variables under set -u.
  set +u
  # shellcheck disable=SC1091,SC1090
  source /opt/ros/foxy/setup.bash
  # shellcheck disable=SC1090
  source "$LOCAL_WS/install/local_setup.bash"
  set -u

  [[ -x "$LOCAL_PYTHON_EXECUTABLE" ]] || \
    die "找不到 ROS Python: $LOCAL_PYTHON_EXECUTABLE"
  "$LOCAL_PYTHON_EXECUTABLE" -c 'import rclpy' >/dev/null 2>&1 || \
    die "$LOCAL_PYTHON_EXECUTABLE 无法导入 rclpy；不要用 Miniconda Python 启动 ROS"

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$LOCAL_DDS_CONFIG
  export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

  local prefix="$LOCAL_WS/install/scan_planner"
  [[ -f "$prefix/share/scan_planner/rviz/default_real.rviz" ]] || \
    die "缺少 default_real.rviz，请先执行 tools 中记录的 colcon build"
  if clicked_waypoints_enabled; then
    [[ -x "$prefix/lib/scan_planner/clicked_waypoint_recorder.py" ]] || \
      die "缺少 clicked_waypoint_recorder.py，请先编译 scan_planner"
  fi
  [[ -x /opt/ros/foxy/bin/rviz2 ]] || die "找不到 rviz2"
}

restart=0
for arg in "$@"; do
  case "$arg" in
    --restart) restart=1 ;;
    -h|--help)
      cat <<'EOF'
用法: start_scan_stack.sh [--restart]

在工作站运行。Go2 的 ROS/DDS 只走 eth0，工作站的 ROS/DDS 只走 lo；
跨机可视化状态和 RViz 目标只通过 Wi-Fi 上的 TCP 9000 桥传输。
默认拒绝覆盖已运行的服务；使用 --restart 会先清理两端服务和旧 Wi-Fi relay。
navi_mode=1 时默认在独立终端打开 Publish Point 路径点记录器，按 s/q 手动保存。
本脚本不发送 1003 StopMove；停车使用遥控器或 StopMove。

环境变量:
  SCAN_CLICKED_WAYPOINTS_ENABLE=auto   auto/1/0，默认仅 navi_mode=1 启用
  SCAN_CLICKED_WAYPOINTS_FILE=...      默认 foxy_ws/maps/clicked_waypoints.yaml
  SCAN_CLICKED_WAYPOINTS_Z=0.4         点击点未到达前使用的固定高度
  SCAN_CLICKED_WAYPOINTS_DRIVE=1       默认点选后发布导航目标；0 表示只记录
  SCAN_CAMERA_ENABLE=1                 1/0，默认把 Go2 前置相机接入 RViz
  SCAN_CAMERA_URL=...                  默认 http://192.168.76.1:8765/frame.jpg
  SCAN_CAMERA_FPS=15                   相机显示帧率，约 94 KB/帧
EOF
      exit 0
      ;;
    *) die "未知参数: $arg" ;;
  esac
done

command -v ssh >/dev/null 2>&1 || die "找不到 ssh"
[[ -x "$STOP_SCRIPT" ]] || die "找不到可执行的停止脚本: $STOP_SCRIPT"

ssh -o ConnectTimeout=5 "$GO2_HOST" true || die "无法 SSH 连接 $GO2_HOST"
check_dds_config
remote_run "ros2 daemon stop >/dev/null 2>&1 || true"

if (( restart == 1 )); then
  printf '重启模式：无条件先停止两端已有服务...\n'
  "$STOP_SCRIPT"
elif remote_stack_running || local_rviz_running; then
  die "检测到已有服务。先运行 stop_scan_stack.sh，或使用 --restart"
fi

check_local_workspace
CLEANUP_ON_FAILURE=1
ros2 daemon stop >/dev/null 2>&1 || true
scp -q "$SCRIPT_DIR/scan_wifi_bridge.py" "$GO2_HOST:$REMOTE_ROOT/tools/scan_wifi_bridge.py" || \
  die "无法同步 TCP 桥到 Go2"
# shellcheck disable=SC2029
ssh "$GO2_HOST" "chmod +x '$REMOTE_ROOT/tools/scan_wifi_bridge.py'"

printf '\n[1/8] 启动 Mid-360 驱动\n'
start_remote livox "$REMOTE_GRAPH_WS" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
wait_for_remote_topic /livox/lidar 20 || { show_remote_log "$REMOTE_LOG_DIR/livox.log"; die "Mid-360 未发布 /livox/lidar"; }
wait_for_remote_topic /livox/imu 20 || { show_remote_log "$REMOTE_LOG_DIR/livox.log"; die "Mid-360 未发布 /livox/imu"; }

printf '[2/8] 启动 FAST-LIO2\n'
start_remote fastlio "$REMOTE_ROOT" \
  "ros2 launch fast_lio mapping.launch.py rviz:=false config_path:=$REMOTE_ROOT/src/planner/lio_bridge/config config_file:=mid360_fastlio.yaml tf_topic:=/fastlio/tf"
wait_for_remote_topic /Odometry 40 || { show_remote_log "$REMOTE_LOG_DIR/fastlio.log"; die "FAST-LIO2 未发布 /Odometry"; }
wait_for_remote_log "$REMOTE_LOG_DIR/fastlio.log" "Initialize the map kdtree" 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/fastlio.log"; die "FAST-LIO2 地图初始化未完成"; }

printf '[3/8] 启动 lio_bridge\n'
start_remote bridge "$REMOTE_ROOT" "ros2 launch lio_bridge fastlio_bridge.launch.py"
wait_for_remote_topic /LIO/odom_vehicle 25 || { show_remote_log "$REMOTE_LOG_DIR/bridge.log"; die "bridge 未发布 /LIO/odom_vehicle"; }
wait_for_remote_topic /LIO/clouds_lidar 25 || { show_remote_log "$REMOTE_LOG_DIR/bridge.log"; die "bridge 未发布 /LIO/clouds_lidar"; }
wait_for_remote_log "$REMOTE_LOG_DIR/bridge.log" "rx/s odom=" 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/bridge.log"; die "bridge 未收到 LIO 数据"; }

printf '[4/8] 启动 SCAN-Planner\n'
SCAN_KEYPOINTS_ARG=""
if [[ -n "$KEYPOINTS_FILE" ]]; then
  SCAN_KEYPOINTS_ARG="keypoints_file:=$KEYPOINTS_FILE"
fi
start_remote scan "$REMOTE_ROOT" \
  "ros2 launch scan_planner run.launch.py is_real_world:=true navi_mode:=$NAVI_MODE $SCAN_KEYPOINTS_ARG sensor_type:=lidar controller_mode:=closed_loop"
wait_for_remote_node /scan_planner_node 35 || { show_remote_log "$REMOTE_LOG_DIR/scan.log"; die "SCAN-Planner 节点未启动"; }
wait_for_remote_topic /grid_map/occupancy 35 || { show_remote_log "$REMOTE_LOG_DIR/scan.log"; die "SCAN-Planner 未发布栅格地图"; }
wait_for_remote_log "$REMOTE_LOG_DIR/scan.log" "wait for goal" 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/scan.log"; die "SCAN-Planner 未进入等待目标状态"; }

printf '[5/8] 启动 /cmd_vel -> Unitree Sport 直通桥\n'
start_remote cmd_vel_sport "$REMOTE_ROOT" \
  "ros2 launch cmd_vel_sport_bridge cmd_vel_sport_bridge.launch.py"
wait_for_remote_node /cmd_vel_sport_bridge 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/cmd_vel_sport.log"; die "cmd_vel_sport_bridge 未启动"; }

printf '[6/8] 启动 Wi-Fi TCP 状态桥\n'
start_remote wifi_bridge "$REMOTE_ROOT" \
  "$REMOTE_PYTHON_EXECUTABLE $REMOTE_ROOT/tools/scan_wifi_bridge.py --server --bind 0.0.0.0 --port 9000"
wait_for_remote_log "$REMOTE_LOG_DIR/wifi_bridge.log" "listening on" 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/wifi_bridge.log"; die "Go2 TCP 桥未开始监听 9000"; }
start_local wifi_bridge \
  "$LOCAL_PYTHON_EXECUTABLE $SCRIPT_DIR/scan_wifi_bridge.py --client --host $GO2_WIFI_HOST --port 9000"
wait_for_local_log "$LOCAL_LOG_DIR/wifi_bridge.log" "Wi-Fi clock offset" 25 || \
  { tail -n 80 "$LOCAL_LOG_DIR/wifi_bridge.log" >&2 2>/dev/null || true; die "工作站 TCP 桥未连上 Go2"; }
wait_local_topic /tf 25 || die "本地 TCP 桥未发布 /tf"
wait_local_topic /LIO/clouds_lidar 30 || die "本地 TCP 桥未转发 /LIO/clouds_lidar"
wait_local_topic /grid_map/occupancy 30 || die "本地 TCP 桥未转发 /grid_map/occupancy"
if (( CAMERA_ENABLED == 1 )); then
  front_camera_start "$LOCAL_WS" "$LOCAL_PYTHON_EXECUTABLE" "$SCRIPT_DIR" "$LOCAL_LOG_DIR"
  if wait_local_topic "$CAMERA_TOPIC" 8; then
    printf '前置相机已接入: %s (%s Hz, 独立进程直连)\n' "$CAMERA_URL" "$CAMERA_FPS"
  else
    printf '警告: 前置相机暂未出图，检查 Go2 的 %s\n' "$CAMERA_URL" >&2
  fi
fi

printf '\n[7/8] 打开 Publish Point 路径点记录器终端\n'
if clicked_waypoints_enabled; then
  recorder_args=(
    "$LOCAL_PYTHON_EXECUTABLE"
    "$LOCAL_WS/install/scan_planner/lib/scan_planner/clicked_waypoint_recorder.py"
    --output "$CLICKED_WAYPOINTS_FILE"
    --z "$CLICKED_WAYPOINTS_Z"
  )
  if [[ "$CLICKED_WAYPOINTS_DRIVE" == "1" ]]; then
    recorder_args+=(--drive)
  fi
  printf -v recorder_command '%q ' "${recorder_args[@]}"
  mkdir -p "$(dirname "$CLICKED_WAYPOINTS_FILE")"
  recorder_log="$LOCAL_LOG_DIR/clicked_waypoints.log"
  : > "$recorder_log"
  terminal_body=$(cat <<EOF
set -o pipefail
${recorder_command% } 2>&1 | tee $(printf '%q' "$recorder_log")
status=\${PIPESTATUS[0]}
printf '\n记录器已退出 (code %s)。按回车关闭窗口...\n' "\$status"
read -r _
exit "\$status"
EOF
)
  if ! open_local_terminal "SCAN Publish Point Recorder" "$terminal_body"; then
    printf '警告: 无法打开记录器终端，改为后台记录；本次无法用 s/q 手动保存。\n' >&2
    start_local clicked_waypoints "${recorder_command% } --no-keyboard"
  fi
  wait_for_local_log "$recorder_log" \
    "Click waypoints on /clicked_point" 15 || {
    tail -n 80 "$recorder_log" >&2 2>/dev/null || true
    die "Publish Point 路径点记录器未启动"
  }
  printf '路径点输出: %s（在记录器终端按 s 或 q 保存）\n' "$CLICKED_WAYPOINTS_FILE"
else
  printf '已跳过（SCAN_CLICKED_WAYPOINTS_ENABLE=%s, navi_mode=%s）\n' \
    "$CLICKED_WAYPOINTS_ENABLE" "$NAVI_MODE"
fi

printf '\n[8/8] 打开工作站 RViz\n'
printf '启动完成。远端日志:\n'
printf '  ssh %s tail -f %s/{livox,fastlio,bridge,scan,cmd_vel_sport}.log\n' "$GO2_HOST" "$REMOTE_LOG_DIR"
printf '本地 TCP 桥日志: %s/wifi_bridge.log\n' "$LOCAL_LOG_DIR"
if clicked_waypoints_enabled; then
  printf '路径点记录器日志: %s/clicked_waypoints.log\n' "$LOCAL_LOG_DIR"
fi
printf '关闭 RViz 不会停止 Go2 服务；停止全部服务: %s\n\n' "$STOP_SCRIPT"
printf '仅重开 RViz: %s\n\n' "$SCRIPT_DIR/restart_rviz.sh"

CLEANUP_ON_FAILURE=0
cd "$LOCAL_WS"
exec ros2 launch scan_planner rviz.launch.py rviz_config:=real

#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
STOP_SCRIPT="$SCRIPT_DIR/stop_scan_stack.sh"

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
GO2_WIFI_HOST=${GO2_WIFI_HOST:-192.168.76.1}
LOCAL_DDS_CONFIG=${SCAN_LOCAL_DDS_CONFIG:-$HOME/.config/ros2/cyclonedds.xml}
REMOTE_DDS_CONFIG=${SCAN_REMOTE_DDS_CONFIG:-$REMOTE_HOME/.config/ros2/cyclonedds.xml}

export ROS_DOMAIN_ID

CLEANUP_ON_FAILURE=0

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

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

wait_for_local_topic() {
  local topic=$1
  local timeout_s=$2
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if timeout 4 ros2 topic list --no-daemon --spin-time 1 2>/dev/null | grep -Fxq -- "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
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
  pgrep -f 'rviz2 .*default.*\.rviz|ros2 launch scan_planner rviz\.launch\.py|scan_wifi_bridge\.py --client' >/dev/null 2>&1
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

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$LOCAL_DDS_CONFIG
  export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

  local prefix="$LOCAL_WS/install/scan_planner"
  [[ -f "$prefix/share/scan_planner/rviz/default_real.rviz" ]] || \
    die "缺少 default_real.rviz，请先执行 tools 中记录的 colcon build"
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
本脚本不发送 1003 StopMove；停车使用遥控器或 StopMove。
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
ssh "$GO2_HOST" "chmod +x '$REMOTE_ROOT/tools/scan_wifi_bridge.py'"

printf '\n[1/7] 启动 Mid-360 驱动\n'
start_remote livox "$REMOTE_GRAPH_WS" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
wait_for_remote_topic /livox/lidar 20 || { show_remote_log "$REMOTE_LOG_DIR/livox.log"; die "Mid-360 未发布 /livox/lidar"; }
wait_for_remote_topic /livox/imu 20 || { show_remote_log "$REMOTE_LOG_DIR/livox.log"; die "Mid-360 未发布 /livox/imu"; }

printf '[2/7] 启动 FAST-LIO2\n'
start_remote fastlio "$REMOTE_ROOT" \
  "ros2 launch fast_lio mapping.launch.py rviz:=false config_path:=$REMOTE_ROOT/src/planner/lio_bridge/config config_file:=mid360_fastlio.yaml tf_topic:=/fastlio/tf"
wait_for_remote_topic /Odometry 40 || { show_remote_log "$REMOTE_LOG_DIR/fastlio.log"; die "FAST-LIO2 未发布 /Odometry"; }
wait_for_remote_log "$REMOTE_LOG_DIR/fastlio.log" "Initialize the map kdtree" 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/fastlio.log"; die "FAST-LIO2 地图初始化未完成"; }

printf '[3/7] 启动 lio_bridge\n'
start_remote bridge "$REMOTE_ROOT" "ros2 launch lio_bridge fastlio_bridge.launch.py"
wait_for_remote_topic /LIO/odom_vehicle 25 || { show_remote_log "$REMOTE_LOG_DIR/bridge.log"; die "bridge 未发布 /LIO/odom_vehicle"; }
wait_for_remote_topic /LIO/clouds_lidar 25 || { show_remote_log "$REMOTE_LOG_DIR/bridge.log"; die "bridge 未发布 /LIO/clouds_lidar"; }
wait_for_remote_log "$REMOTE_LOG_DIR/bridge.log" "rx/s odom=" 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/bridge.log"; die "bridge 未收到 LIO 数据"; }

printf '[4/7] 启动 SCAN-Planner\n'
start_remote scan "$REMOTE_ROOT" \
  "ros2 launch scan_planner run.launch.py is_real_world:=true navi_mode:=1 sensor_type:=lidar controller_mode:=closed_loop"
wait_for_remote_node /scan_planner_node 35 || { show_remote_log "$REMOTE_LOG_DIR/scan.log"; die "SCAN-Planner 节点未启动"; }
wait_for_remote_topic /grid_map/occupancy 35 || { show_remote_log "$REMOTE_LOG_DIR/scan.log"; die "SCAN-Planner 未发布栅格地图"; }
wait_for_remote_log "$REMOTE_LOG_DIR/scan.log" "wait for goal" 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/scan.log"; die "SCAN-Planner 未进入等待目标状态"; }

printf '[5/7] 启动 /cmd_vel -> Unitree Sport 直通桥\n'
start_remote cmd_vel_sport "$REMOTE_ROOT" \
  "ros2 launch cmd_vel_sport_bridge cmd_vel_sport_bridge.launch.py"
wait_for_remote_node /cmd_vel_sport_bridge 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/cmd_vel_sport.log"; die "cmd_vel_sport_bridge 未启动"; }

printf '[6/7] 启动 Wi-Fi TCP 状态桥\n'
start_remote wifi_bridge "$REMOTE_ROOT" \
  "python3 $REMOTE_ROOT/tools/scan_wifi_bridge.py --server --bind 0.0.0.0 --port 9000"
wait_for_remote_log "$REMOTE_LOG_DIR/wifi_bridge.log" "listening on" 25 || \
  { show_remote_log "$REMOTE_LOG_DIR/wifi_bridge.log"; die "Go2 TCP 桥未开始监听 9000"; }
start_local wifi_bridge \
  "python3 $SCRIPT_DIR/scan_wifi_bridge.py --client --host $GO2_WIFI_HOST --port 9000"
wait_for_local_log "$LOCAL_LOG_DIR/wifi_bridge.log" "Wi-Fi clock offset" 25 || \
  { tail -n 80 "$LOCAL_LOG_DIR/wifi_bridge.log" >&2 2>/dev/null || true; die "工作站 TCP 桥未连上 Go2"; }
wait_for_local_topic /tf 25 || die "本地 TCP 桥未发布 /tf"
wait_for_local_topic /LIO/clouds_lidar 30 || die "本地 TCP 桥未转发 /LIO/clouds_lidar"
wait_for_local_topic /grid_map/occupancy 30 || die "本地 TCP 桥未转发 /grid_map/occupancy"

printf '\n[7/7] 打开工作站 RViz\n'
printf '启动完成。远端日志:\n'
printf '  ssh %s tail -f %s/{livox,fastlio,bridge,scan,cmd_vel_sport}.log\n' "$GO2_HOST" "$REMOTE_LOG_DIR"
printf '本地 TCP 桥日志: %s/wifi_bridge.log\n' "$LOCAL_LOG_DIR"
printf '关闭 RViz 不会停止 Go2 服务；停止全部服务: %s\n\n' "$STOP_SCRIPT"
printf '仅重开 RViz: %s\n\n' "$SCRIPT_DIR/restart_rviz.sh"

CLEANUP_ON_FAILURE=0
cd "$LOCAL_WS"
exec ros2 launch scan_planner rviz.launch.py rviz_config:=real

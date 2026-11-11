#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/stack_lib.sh
source "$SCRIPT_DIR/stack_lib.sh"
REMOTE_HOME=${SCAN_REMOTE_HOME:-/home/unitree}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-$REMOTE_HOME/SCAN-Planner}
REMOTE_GRAPH_WS=${SCAN_REMOTE_GRAPH_WS:-/unitree/module/graph_pid_ws}
REMOTE_FASTLIO_WS=${SCAN_REMOTE_FASTLIO_WS:-$REMOTE_HOME/fastlio_ws}
REMOTE_DDS_CONFIG=${SCAN_REMOTE_DDS_CONFIG:-$REMOTE_HOME/.config/ros2/cyclonedds.xml}
remote_stop_group() {
  local name=$1
  local pattern=$2
  local script
  script=$(cat <<'REMOTE'
set -uo pipefail
own=$$
ownpg=$(ps -o pgid= -p "$$" | tr -d ' ')

mapfile -t pgids < <(
  ps -eo pid=,pgid=,args= | awk \
    -v own="$own" \
    -v ownpg="$ownpg" \
    -v pattern="$PATTERN" \
    '$1 != own && $2 != ownpg && $0 ~ pattern {print $2}' | sort -u
)

if ((${#pgids[@]} == 0)); then
  printf '  %s: 无运行进程\n' "$NAME"
  exit 0
fi

printf '  %s: PGID %s\n' "$NAME" "${pgids[*]}"
for pgid in "${pgids[@]}"; do
  kill -TERM -- "-$pgid" 2>/dev/null || true
done

sleep 1
for pgid in "${pgids[@]}"; do
  if kill -0 -- "-$pgid" 2>/dev/null; then
    printf '  %s: PGID %s 未退出，发送 SIGKILL\n' "$NAME" "$pgid"
    kill -KILL -- "-$pgid" 2>/dev/null || true
  fi
done
REMOTE
)

  # shellcheck disable=SC2029
  ssh "$GO2_HOST" "NAME=$(printf '%q' "$name") PATTERN=$(printf '%q' "$pattern") bash -s" \
    <<<"$script"
}

remote_save_map() {
  local script
  script=$(cat <<REMOTE
set +u
source /opt/ros/foxy/setup.bash
source "$REMOTE_GRAPH_WS/install/local_setup.bash" 2>/dev/null || true
source "$REMOTE_FASTLIO_WS/install/local_setup.bash" 2>/dev/null || true
source "$REMOTE_ROOT/install/local_setup.bash" 2>/dev/null || true
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$REMOTE_DDS_CONFIG
export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}
if ! pgrep -f 'fastlio_mapping|fast_lio mapping\.launch\.py' >/dev/null 2>&1; then
  exit 0
fi
mkdir -p "$REMOTE_ROOT/maps"
timeout 60 ros2 service call /map_save std_srvs/srv/Trigger {}
REMOTE
)
  ssh "$GO2_HOST" bash -s <<<"$script"
}

printf '停止工作站 RViz...\n'
stop_local_pgids "RViz" 'rviz2 .*default.*\.rviz|ros2 launch scan_planner rviz\.launch\.py'

printf '停止工作站 Publish Point 记录器...\n'
stop_local_pgids "Publish Point recorder" 'clicked_waypoint_recorder\.py'

printf '停止工作站 TCP 状态桥...\n'
stop_local_pgids "Wi-Fi TCP client" 'scan_wifi_bridge\.py --client'

printf '停止工作站 YOLO 叠框...\n'
stop_local_pgids "YOLO overlay" 'yolo_overlay_node( |$)'

printf '停止工作站前置相机...\n'
front_camera_stop

printf '停止工作站语义分割...\n'
"$SCRIPT_DIR/stop_semantic_scene.sh"

printf '停止工作站 VLM 节点...\n'
stop_local_pgids "VLM pixel shadow" 'vlm_pixel_shadow\.py'
stop_local_pgids "llama.cpp SSH tunnel" \
  'ssh .* -L 8001:127\.0\.0\.1:8001 AnyaCoder@10\.126\.62\.90'

printf '停止 Go2 服务...\n'
if ! ssh -o ConnectTimeout=5 "$GO2_HOST" true; then
  printf '错误: 无法 SSH 连接 %s；远端服务未清理\n' "$GO2_HOST" >&2
  exit 1
fi

# 先从可视化开始，再按依赖顺序停止规划、bridge、LIO 和雷达驱动。
remote_stop_group "YOLO detection server" 'detection_http\.py'
remote_stop_group "YOLO detector" '[y]olo_trt'
remote_stop_group "RViz (Go2)" \
  'rviz2 .*default.*\.rviz|ros2 launch scan_planner rviz\.launch\.py'
remote_stop_group "Wi-Fi TCP server" \
  'scan_wifi_bridge\.py --server'
remote_stop_group "旧 Wi-Fi DDS relay" \
  'native_wifi_relay_cpp\.new --(source|wifi)'
remote_stop_group "SCAN-Planner" \
  'scan_planner_node|closed_loop_controller|open_loop_controller|go2_robot_state_publisher|cmd_vel_sport_bridge|scan_planner run\.launch\.py|cmd_vel_sport_bridge\.launch\.py'
remote_stop_group "lio_bridge" \
  'lio_bridge_node|lio_bridge fastlio_bridge\.launch\.py'
printf '保存 FAST-LIO2 全局点云...\n'
remote_save_map || printf '  警告: /map_save 调用失败\n' >&2
remote_stop_group "FAST-LIO2" \
  'fastlio_mapping|fast_lio mapping\.launch\.py'
remote_stop_group "Livox driver" \
  'livox_ros_driver2_node|livox_ros_driver2 msg_MID360_launch\.py'

# 清掉可能仍挂着的本地 SSH 启动包装进程，不匹配普通交互式 SSH。
printf '清理工作站 SSH 启动包装...\n'
go2_host_pattern=$(printf '%s' "$GO2_HOST" | sed 's/\./\\./g')
ssh_pattern="^ssh .*${go2_host_pattern} .*(scan_planner|lio_bridge|fast_lio|fastlio|livox_ros_driver2)"
stop_local_pgids "SSH launcher" "$ssh_pattern"

printf '清理完成。\n'

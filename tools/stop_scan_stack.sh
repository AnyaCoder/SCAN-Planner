#!/usr/bin/env bash
set -uo pipefail

GO2_HOST=${GO2_HOST:-go2-wifi}

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

  ssh "$GO2_HOST" "NAME=$(printf '%q' "$name") PATTERN=$(printf '%q' "$pattern") bash -s" \
    <<<"$script"
}

stop_local_pgids() {
  local label=$1
  local pattern=$2
  local ownpg
  ownpg=$(ps -o pgid= -p "$$" | tr -d ' ')
  local pgids=()

  mapfile -t pgids < <(
    ps -eo pid=,pgid=,args= | awk \
      -v own="$$" \
      -v ownpg="$ownpg" \
      -v pattern="$pattern" \
      '$1 != own && $2 != ownpg && $0 ~ pattern {print $2}' | sort -u
  )

  if ((${#pgids[@]} == 0)); then
    printf '  %s: 无运行进程\n' "$label"
    return 0
  fi

  printf '  %s: PGID %s\n' "$label" "${pgids[*]}"
  local pgid
  for pgid in "${pgids[@]}"; do
    kill -TERM -- "-$pgid" 2>/dev/null || true
  done
  sleep 1
  for pgid in "${pgids[@]}"; do
    if kill -0 -- "-$pgid" 2>/dev/null; then
      printf '  %s: PGID %s 未退出，发送 SIGKILL\n' "$label" "$pgid"
      kill -KILL -- "-$pgid" 2>/dev/null || true
    fi
  done
}

printf '停止工作站 RViz...\n'
stop_local_pgids "RViz" 'rviz2 .*default.*\.rviz|ros2 launch scan_planner rviz\.launch\.py'

printf '停止工作站 TCP 状态桥...\n'
stop_local_pgids "Wi-Fi TCP client" 'scan_wifi_bridge\.py --client'

printf '停止 Go2 服务...\n'
if ! ssh -o ConnectTimeout=5 "$GO2_HOST" true; then
  printf '错误: 无法 SSH 连接 %s；远端服务未清理\n' "$GO2_HOST" >&2
  exit 1
fi

# 先从可视化开始，再按依赖顺序停止规划、bridge、LIO 和雷达驱动。
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
remote_stop_group "FAST-LIO2" \
  'fastlio_mapping|fast_lio mapping\.launch\.py'
remote_stop_group "Livox driver" \
  'livox_ros_driver2_node|livox_ros_driver2 msg_MID360_launch\.py'

# 清掉可能仍挂着的本地 SSH 启动包装进程，不匹配普通交互式 SSH。
printf '清理工作站 SSH 启动包装...\n'
stop_local_pgids "SSH launcher" \
  '^ssh .*go2-wifi .*(scan_planner|lio_bridge|fast_lio|fastlio|livox_ros_driver2)'

printf '清理完成。\n'

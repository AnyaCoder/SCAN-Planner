#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
LOCAL_WS=${SCAN_LOCAL_WS:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
LOCAL_LOG_DIR=${SCAN_LOCAL_LOG_DIR:-/tmp/scan_planner_local_logs}
LOCAL_DDS_CONFIG=${SCAN_LOCAL_DDS_CONFIG:-$HOME/.config/ros2/cyclonedds.xml}
GO2_WIFI_HOST=${GO2_WIFI_HOST:-192.168.76.1}
BRIDGE_PORT=${SCAN_BRIDGE_PORT:-9000}
BRIDGE_LOG="$LOCAL_LOG_DIR/wifi_bridge.log"

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
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
    return 0
  fi

  printf '停止 %s: PGID %s\n' "$label" "${pgids[*]}"
  local pgid
  for pgid in "${pgids[@]}"; do
    kill -TERM -- "-$pgid" 2>/dev/null || true
  done
  sleep 1
  for pgid in "${pgids[@]}"; do
    if kill -0 -- "-$pgid" 2>/dev/null; then
      printf '  %s 未退出，发送 SIGKILL\n' "$label"
      kill -KILL -- "-$pgid" 2>/dev/null || true
    fi
  done
}

bridge_running() {
  pgrep -f 'scan_wifi_bridge\.py --client' >/dev/null 2>&1
}

start_bridge() {
  mkdir -p "$LOCAL_LOG_DIR"
  printf '启动本地 TCP 客户端...\n'
  setsid -f bash -c \
    "cd $(printf '%q' "$LOCAL_WS") && exec python3 $(printf '%q' "$SCRIPT_DIR/scan_wifi_bridge.py") --client --host $(printf '%q' "$GO2_WIFI_HOST") --port $(printf '%q' "$BRIDGE_PORT")" \
    >"$BRIDGE_LOG" 2>&1 </dev/null
}

wait_bridge_connected() {
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if [[ -f "$BRIDGE_LOG" ]] && grep -Fq 'Wi-Fi clock offset' "$BRIDGE_LOG"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_local_topic() {
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

wait_world_body() {
  local timeout_s=$1
  local deadline=$((SECONDS + timeout_s))
  local tmp
  tmp=$(mktemp)
  while (( SECONDS < deadline )); do
    timeout 3 ros2 run tf2_ros tf2_echo world body >"$tmp" 2>/dev/null || true
    if grep -q 'Translation:' "$tmp"; then
      rm -f "$tmp"
      return 0
    fi
    sleep 1
  done
  rm -f "$tmp"
  return 1
}

flow_ready() {
  wait_local_topic /tf 8 && \
    wait_local_topic /LIO/clouds_lidar 8 && \
    wait_world_body 12
}

[[ -n "${DISPLAY:-}" ]] || die "当前终端没有 DISPLAY，无法打开 RViz"
[[ -f /opt/ros/foxy/setup.bash ]] || die "缺少 /opt/ros/foxy/setup.bash"
[[ -f "$LOCAL_WS/install/local_setup.bash" ]] || die "缺少 $LOCAL_WS/install/local_setup.bash"
[[ -f "$LOCAL_DDS_CONFIG" ]] || die "缺少 DDS 配置: $LOCAL_DDS_CONFIG"
grep -Fq -- 'name="lo"' "$LOCAL_DDS_CONFIG" || die "工作站 DDS 未绑定 lo: $LOCAL_DDS_CONFIG"

set +u
source /opt/ros/foxy/setup.bash
source "$LOCAL_WS/install/local_setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$LOCAL_DDS_CONFIG
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
ros2 daemon stop >/dev/null 2>&1 || true

[[ -f "$LOCAL_WS/install/scan_planner/share/scan_planner/rviz/default_real.rviz" ]] || \
  die "缺少 default_real.rviz，请先编译 scan_planner"

stop_local_pgids "旧 RViz" 'rviz2 .*default.*\.rviz|ros2 launch scan_planner rviz\.launch\.py'

if ! bridge_running; then
  start_bridge
fi

if ! flow_ready; then
  printf '数据流未就绪，重启本地 TCP 客户端...\n'
  stop_local_pgids "本地 TCP 客户端" 'scan_wifi_bridge\.py --client'
  start_bridge
  wait_bridge_connected || {
    tail -n 80 "$BRIDGE_LOG" >&2 2>/dev/null || true
    die "本地 TCP 客户端未连上 Go2 的 $GO2_WIFI_HOST:$BRIDGE_PORT"
  }
  flow_ready || die "本地 TCP 客户端已连接，但数据流仍未恢复"
fi

printf 'TCP 客户端已就绪，打开 RViz。\n'
cd "$LOCAL_WS"
exec ros2 launch scan_planner rviz.launch.py rviz_config:=real

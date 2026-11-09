#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/stack_lib.sh
source "$SCRIPT_DIR/stack_lib.sh"

GO2_HOST=${GO2_HOST:-go2-wifi}
REMOTE_HOME=${SCAN_REMOTE_HOME:-/home/unitree}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-$REMOTE_HOME/SCAN-Planner}
LOCAL_LOG_DIR=${SCAN_LOCAL_LOG_DIR:-/tmp/scan_planner_local_logs}
LOCAL_WS=${SCAN_LOCAL_WS:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
LOCAL_PYTHON_EXECUTABLE=${SCAN_PYTHON:-/usr/bin/python3}

front_camera_defaults || true

printf '停止工作站 YOLO 叠框...\n'
stop_local_pgids "YOLO overlay" 'yolo_overlay_node( |$)'

printf '停止 Go2 YOLO 服务...\n'
if ssh -o ConnectTimeout=5 "$GO2_HOST" true; then
  ssh "$GO2_HOST" bash -s <<'EOF'
pkill -TERM -f 'detection_http\.py' 2>/dev/null || true
pkill -TERM -f '[y]olo_trt' 2>/dev/null || true
sleep 1
pkill -KILL -f 'detection_http\.py' 2>/dev/null || true
pkill -KILL -f '[y]olo_trt' 2>/dev/null || true
EOF
else
  printf '错误: 无法 SSH 连接 %s；远端 YOLO 未清理\n' "$GO2_HOST" >&2
  exit 1
fi

if front_camera_relay_running && ! front_camera_decoder_running; then
  printf '恢复普通相机解码...\n'
  front_camera_start "$LOCAL_WS" "$LOCAL_PYTHON_EXECUTABLE" "$SCRIPT_DIR" "$LOCAL_LOG_DIR"
fi

printf 'YOLO 叠框已停止。\n'

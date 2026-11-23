#!/usr/bin/env bash
# Keep a five-second dual-LiDAR window in Go2 memory and render on demand.
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REMOTE_SSH=${SCAN_REMOTE_SSH:-go2-wifi}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-/home/unitree/SCAN-Planner}
REMOTE_OUTPUT_DIR=$REMOTE_ROOT/debug/pointcloud_views
REMOTE_TOOLS_DIR=$REMOTE_ROOT/debug/tools
REMOTE_PNG=$REMOTE_OUTPUT_DIR/mid360_topdown_live.png
REMOTE_JSON=$REMOTE_OUTPUT_DIR/mid360_topdown_live.json
REMOTE_TRIGGER=$REMOTE_OUTPUT_DIR/mid360_topdown_request
REMOTE_READY=$REMOTE_OUTPUT_DIR/mid360_topdown_ready.json
REMOTE_LOG=$REMOTE_OUTPUT_DIR/mid360_topdown_live.log

ssh -o BatchMode=yes "$REMOTE_SSH" \
  "mkdir -p '$REMOTE_OUTPUT_DIR' '$REMOTE_TOOLS_DIR'"
scp -q "$SCRIPT_DIR/capture_mid360_topdown.py" \
  "$SCRIPT_DIR/capture_mid360_topdown.sh" \
  "$REMOTE_SSH:$REMOTE_TOOLS_DIR/"

window_status=$(
  ssh -o BatchMode=yes "$REMOTE_SSH" "
    if ! pgrep -f '[c]apture_mid360_topdown\.py --rolling' >/dev/null 2>&1; then
      echo missing
    elif [[ -s '$REMOTE_READY' ]]; then
      echo ready
    else
      echo warming
    fi
  "
)

if [[ "$window_status" == "missing" ]]; then
  ssh -o BatchMode=yes "$REMOTE_SSH" "
    rm -f '$REMOTE_READY' '$REMOTE_TRIGGER'
    setsid -f bash '$REMOTE_TOOLS_DIR/capture_mid360_topdown.sh' \
      --rolling --duration 5 \
      --trigger-file '$REMOTE_TRIGGER' \
      --ready-file '$REMOTE_READY' \
      --resolution 0.05 --range 5 --max-height 1.20 \
      --style green_points \
      --output '$REMOTE_PNG' \
      >'$REMOTE_LOG' 2>&1 </dev/null
  "
fi

deadline=$((SECONDS + 15))
while (( SECONDS < deadline )); do
  if ssh -o BatchMode=yes "$REMOTE_SSH" \
    "test -s '$REMOTE_READY'"; then
    printf '5 s 双雷达点云窗口已就绪，按需渲染: %s\n' "$REMOTE_PNG"
    exit 0
  fi
  sleep 1
done

printf '滚动俯视图启动失败，远端日志: %s\n' "$REMOTE_LOG" >&2
ssh -o BatchMode=yes "$REMOTE_SSH" \
  "tail -n 80 '$REMOTE_LOG' 2>/dev/null || true" >&2 || true
exit 1

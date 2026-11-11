#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/stack_lib.sh
source "$SCRIPT_DIR/stack_lib.sh"

LOCAL_WS=${SCAN_LOCAL_WS:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
LOCAL_LOG_DIR=${SCAN_LOCAL_LOG_DIR:-/tmp/scan_planner_local_logs}
LOCAL_DDS_CONFIG=${SCAN_LOCAL_DDS_CONFIG:-$HOME/.config/ros2/cyclonedds.xml}
PYTHON_EXECUTABLE=${SCAN_PYTHON:-/usr/bin/python3}
SCRIPT=${SCAN_VLM_PIXEL_SCRIPT:-$LOCAL_WS/src/planner/plan_manage/scripts/vlm_pixel_shadow.py}
INTERVAL=${SCAN_VLM_PIXEL_INTERVAL:-8}
MAX_DISTANCE=${SCAN_VLM_PIXEL_MAX_DISTANCE:-5.5}
DRIVE=${SCAN_VLM_DRIVE:-1}
API_BACKEND=${SCAN_VLM_API_BACKEND:-llama_cpp}
API_URL=${SCAN_VLM_API_URL:-}
API_MODEL=${SCAN_VLM_MODEL:-}
LLAMA_SSH_HOST=${SCAN_LLAMA_SSH_HOST:-AnyaCoder@10.126.62.90}
LLAMA_LOCAL_PORT=${SCAN_LLAMA_LOCAL_PORT:-8001}
LLAMA_REMOTE_PORT=${SCAN_LLAMA_REMOTE_PORT:-8001}
LLAMA_HEALTH_URL=${SCAN_LLAMA_HEALTH_URL:-http://127.0.0.1:$LLAMA_LOCAL_PORT/health}

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

[[ -f "$SCRIPT" ]] || die "缺少 VLM 像素影子节点: $SCRIPT"
[[ -f "$LOCAL_DDS_CONFIG" ]] || die "缺少 DDS 配置: $LOCAL_DDS_CONFIG"
mkdir -p "$LOCAL_LOG_DIR"

llama_health() {
  curl -fsS --max-time 3 "$LLAMA_HEALTH_URL" >/dev/null 2>&1
}

case "$API_BACKEND" in
  llama_cpp)
    API_URL=${SCAN_VLM_API_URL:-http://127.0.0.1:8001/v1/chat/completions}
    API_MODEL=${SCAN_VLM_MODEL:-unsloth/Qwen3.5-27B-GGUF}
    if ! llama_health; then
      ssh -o ConnectTimeout=5 -o BatchMode=yes "$LLAMA_SSH_HOST" true || \
        die "无法 SSH 连接 llama.cpp 主机 $LLAMA_SSH_HOST"
      stop_local_pgids "llama.cpp SSH tunnel" \
        "ssh .* -L $LLAMA_LOCAL_PORT:127\\.0\\.0\\.1:$LLAMA_REMOTE_PORT $LLAMA_SSH_HOST" \
        quiet
      setsid -f ssh \
        -o BatchMode=yes \
        -o ExitOnForwardFailure=yes \
        -o ServerAliveInterval=20 \
        -o ServerAliveCountMax=3 \
        -N -L "$LLAMA_LOCAL_PORT:127.0.0.1:$LLAMA_REMOTE_PORT" \
        "$LLAMA_SSH_HOST" \
        >"$LOCAL_LOG_DIR/llama_cpp_tunnel.log" 2>&1 </dev/null
      for _ in 1 2 3 4 5; do
        llama_health && break
        sleep 1
      done
      llama_health || die "llama.cpp SSH 隧道未就绪"
    fi
    export VLM_API_KEY=local-llama-cpp
    ;;
  remote)
    ssh -o ConnectTimeout=5 -o BatchMode=yes "$GO2_HOST" true || \
      die "无法 SSH 连接 $GO2_HOST"
    api_key=$(
      ssh -o ConnectTimeout=5 -o BatchMode=yes "$GO2_HOST" \
        'source "$HOME/.env" 2>/dev/null || true; printf %s "${VLM_API_KEY:-}"'
    )
    [[ -n "$api_key" ]] || die "Go2 的 ~/.env 中没有 VLM_API_KEY"
    export VLM_API_KEY="$api_key"
    unset api_key
    [[ -n "$API_URL" ]] || die "remote 后端需要 SCAN_VLM_API_URL"
    [[ -n "$API_MODEL" ]] || die "remote 后端需要 SCAN_VLM_MODEL"
    ;;
  *)
    die "SCAN_VLM_API_BACKEND 只能是 llama_cpp 或 remote: $API_BACKEND"
    ;;
esac

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# shellcheck disable=SC1090
source "$LOCAL_WS/install/local_setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$LOCAL_DDS_CONFIG
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

stop_local_pgids "VLM pixel shadow" 'vlm_pixel_shadow\.py' quiet
launcher=(
  "$PYTHON_EXECUTABLE"
  "$SCRIPT"
  --interval "$INTERVAL"
  --max-target-distance "$MAX_DISTANCE"
  --api-url "$API_URL"
  --model "$API_MODEL"
)
if [[ "$API_BACKEND" == "llama_cpp" ]]; then
  launcher+=(--reasoning-effort none --disable-thinking)
fi
if [[ "$DRIVE" != "0" ]]; then
  launcher+=(--drive)
fi
launcher_command=$(printf '%q ' "${launcher[@]}")
setsid -f bash -c \
  "exec $launcher_command" \
  >"$LOCAL_LOG_DIR/vlm_pixel_shadow.log" 2>&1 </dev/null
unset VLM_API_KEY

sleep 1
pgrep -f 'vlm_pixel_shadow\.py' >/dev/null 2>&1 || {
  tail -n 80 "$LOCAL_LOG_DIR/vlm_pixel_shadow.log" >&2 2>/dev/null || true
  die "VLM 像素影子节点未启动"
}
if [[ "$DRIVE" == "0" ]]; then
  printf 'VLM 像素节点已启动（不发布运动目标）\n'
else
  printf 'VLM 像素节点已启动（候选点必须经 RViz 面板批准）\n'
fi
printf '模型后端: %s (%s, %s)\n' "$API_BACKEND" "$API_MODEL" "$API_URL"
printf '地图标记: /vlm_pixel_shadow\n'
printf '像素标注图: /go2/front_camera/vlm_point_overlay\n'
printf '日志: %s/vlm_pixel_shadow.log\n' "$LOCAL_LOG_DIR"

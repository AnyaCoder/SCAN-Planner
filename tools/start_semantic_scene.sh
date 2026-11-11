#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/stack_lib.sh
source "$SCRIPT_DIR/stack_lib.sh"
LOCAL_WS=${SCAN_LOCAL_WS:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
REMOTE_HOME=${SCAN_REMOTE_HOME:-/home/unitree}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-$REMOTE_HOME/SCAN-Planner}
REMOTE_LOG_DIR=${SCAN_REMOTE_LOG_DIR:-/tmp/scan_planner_logs}
LOCAL_LOG_DIR=${SCAN_LOCAL_LOG_DIR:-/tmp/scan_planner_local_logs}
ENGINE=${SCAN_SEMANTIC_ENGINE:-$REMOTE_ROOT/perception/engines/segformer_b0_ade_512_fp16.engine}
METADATA=${SCAN_SEMANTIC_METADATA:-$REMOTE_ROOT/perception/engines/segformer_b0_ade_512.json}
FPS=${SCAN_SEMANTIC_FPS:-5}
PORT=${SCAN_SEMANTIC_PORT:-8768}
BIND=${SCAN_SEMANTIC_BIND:-0.0.0.0}
LOG="$REMOTE_LOG_DIR/semantic_tensorrt.log"
LOCAL_METADATA="$LOCAL_WS/perception/models/segformer_b0_ade_512.json"

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

[[ -f "$LOCAL_WS/perception/semantic/tensorrt_service.py" ]] || \
  die "缺少 perception/semantic/tensorrt_service.py"
[[ -f "$LOCAL_WS/perception/semantic/service_common.py" ]] || \
  die "缺少 perception/semantic/service_common.py"
[[ -f "$LOCAL_METADATA" ]] || die "缺少模型元数据: $LOCAL_METADATA"

mkdir -p "$LOCAL_LOG_DIR"
command -v ssh >/dev/null 2>&1 || die "找不到 ssh"
command -v scp >/dev/null 2>&1 || die "找不到 scp"
ssh -o ConnectTimeout=5 "$GO2_HOST" true || \
  die "无法 SSH 连接机械狗: $GO2_HOST"
# shellcheck disable=SC2029
ssh "$GO2_HOST" \
  "mkdir -p $(printf '%q' "$REMOTE_ROOT/perception/semantic") $(printf '%q' "$(dirname "$ENGINE")") $(printf '%q' "$REMOTE_LOG_DIR") && test -f $(printf '%q' "$ENGINE")" || \
  die "机械狗上缺少 TensorRT engine: $ENGINE"

scp -q "$LOCAL_WS/perception/semantic/tensorrt_service.py" \
  "$GO2_HOST:$REMOTE_ROOT/perception/semantic/tensorrt_service.py"
scp -q "$LOCAL_WS/perception/semantic/service_common.py" \
  "$GO2_HOST:$REMOTE_ROOT/perception/semantic/service_common.py"
scp -q "$LOCAL_METADATA" "$GO2_HOST:$METADATA"

# shellcheck disable=SC2087
ssh "$GO2_HOST" bash -s <<EOF
set -Eeuo pipefail
pkill -TERM -f '[t]ensorrt_service\.py' 2>/dev/null || true
pkill -TERM -f '[c]pu_service\.py' 2>/dev/null || true
sleep 1
setsid -f bash -c $(printf '%q' "cd $(printf '%q' "$REMOTE_ROOT") && exec python3 perception/semantic/tensorrt_service.py --url http://127.0.0.1:8765/frame.jpg --engine $(printf '%q' "$ENGINE") --metadata $(printf '%q' "$METADATA") --fps $(printf '%q' "$FPS") --bind $(printf '%q' "$BIND") --port $(printf '%q' "$PORT")") \
  >$(printf '%q' "$LOG") 2>&1 </dev/null
EOF

deadline=$((SECONDS + 45))
while (( SECONDS < deadline )); do
  payload=$(ssh -o ConnectTimeout=3 "$GO2_HOST" \
    "curl -fsS --max-time 2 http://127.0.0.1:$PORT/latest.json" \
    2>/dev/null || true)
  if [[ "$payload" == *'"ready":true'* ]]; then
    printf 'TensorRT 语义分割已就绪: http://127.0.0.1:%s/latest.json\n' "$PORT"
    printf '语义叠加图: http://127.0.0.1:%s/overlay.jpg\n' "$PORT"
    exit 0
  fi
  if ! ssh -o ConnectTimeout=3 "$GO2_HOST" \
    "pgrep -f '[t]ensorrt_service\.py' >/dev/null"; then
    break
  fi
  sleep 1
done

# shellcheck disable=SC2029
ssh "$GO2_HOST" "tail -n 80 $(printf '%q' "$LOG") 2>/dev/null || true" >&2
die "语义分割服务未就绪；日志: $LOG"

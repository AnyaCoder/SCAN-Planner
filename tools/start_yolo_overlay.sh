#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/stack_lib.sh
source "$SCRIPT_DIR/stack_lib.sh"

REMOTE_HOME=${SCAN_REMOTE_HOME:-/home/unitree}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-$REMOTE_HOME/SCAN-Planner}
REMOTE_LOG_DIR=${SCAN_REMOTE_LOG_DIR:-/tmp/scan_planner_logs}
LOCAL_LOG_DIR=${SCAN_LOCAL_LOG_DIR:-/tmp/scan_planner_local_logs}
LOCAL_WS=${SCAN_LOCAL_WS:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
REMOTE_PYTHON_EXECUTABLE=${SCAN_REMOTE_PYTHON:-/usr/bin/python3}
OVERLAY_BINARY=${SCAN_YOLO_OVERLAY_BIN:-$LOCAL_WS/install/scan_planner/lib/scan_planner/yolo_overlay_node}
DETECTION_PORT=${SCAN_YOLO_PORT:-8767}
DETECTION_URL=${SCAN_YOLO_URL:-http://$GO2_WIFI_HOST:$DETECTION_PORT/latest_detections.json}
SEMANTIC_URL=${SCAN_SEMANTIC_OVERLAY_URL:-http://$GO2_WIFI_HOST:8768/overlay.jpg}
CAMERA_URL=${SCAN_CAMERA_URL:-http://$GO2_WIFI_HOST:8765/frame.jpg}
YOLO_ENGINE=${SCAN_YOLO_ENGINE:-$REMOTE_ROOT/perception/engines/yolov8s_worldv2_indoor_fp16.engine}
YOLO_NAMES=${SCAN_YOLO_NAMES:-$REMOTE_ROOT/perception/engines/yolov8s_worldv2_indoor.classes.txt}
YOLO_CLASSES_SOURCE=${SCAN_YOLO_CLASSES_SOURCE:-$LOCAL_WS/perception/yolo_world/classes.txt}
YOLO_BINARY=${SCAN_YOLO_BINARY:-$REMOTE_ROOT/perception/yolo_trt/build/yolo_trt}
YOLO_IMAGE=${SCAN_YOLO_IMAGE:-$REMOTE_ROOT/perception/data/latest_detected.jpg}
YOLO_JSON=${SCAN_YOLO_JSON:-$REMOTE_ROOT/perception/data/latest_detections.json}
YOLO_INTERVAL=${SCAN_YOLO_INTERVAL:-0.2}
YOLO_CONFIDENCE=${SCAN_YOLO_CONFIDENCE:-0.25}
DETECTION_STALE=${SCAN_YOLO_STALE:-0.8}
OVERLAY_WIDTH=${SCAN_OVERLAY_WIDTH:-0}

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

front_camera_defaults || die "SCAN_CAMERA_ENABLE 只能是 1/0: $CAMERA_ENABLE"

command -v ssh >/dev/null 2>&1 || die "找不到 ssh"
ssh -o ConnectTimeout=5 "$GO2_HOST" true || die "无法 SSH 连接 $GO2_HOST"
front_camera_relay_running || die "前置相机 relay 未运行，请先启动 SCAN-Planner 相机链路"
[[ -f /opt/ros/foxy/setup.bash ]] || die "缺少 /opt/ros/foxy/setup.bash"
[[ -f "$LOCAL_WS/install/local_setup.bash" ]] || die "缺少 $LOCAL_WS/install/local_setup.bash"
[[ -x "$OVERLAY_BINARY" ]] || \
  die "缺少 $OVERLAY_BINARY，请先编译 scan_planner"
[[ -f "$YOLO_CLASSES_SOURCE" ]] || \
  die "缺少类别表 $YOLO_CLASSES_SOURCE"

# shellcheck disable=SC2029
ssh "$GO2_HOST" "mkdir -p $(printf '%q' "$(dirname "$YOLO_NAMES")") $(printf '%q' "$(dirname "$YOLO_JSON")")" || \
  die "无法创建 Go2 YOLO 目录"
scp -q "$SCRIPT_DIR/detection_http.py" \
  "$GO2_HOST:$REMOTE_ROOT/tools/detection_http.py" || \
  die "无法同步检测 JSON 服务到 Go2"
# shellcheck disable=SC2029
scp -q "$YOLO_CLASSES_SOURCE" "$GO2_HOST:$YOLO_NAMES" || \
  die "无法同步 YOLO 类别表到 Go2"

# shellcheck disable=SC2029,SC2087
ssh "$GO2_HOST" bash -s <<EOF
set -Eeuo pipefail
mkdir -p $(printf '%q' "$REMOTE_LOG_DIR") $(dirname "$(printf '%q' "$YOLO_JSON")")
test -x $(printf '%q' "$YOLO_BINARY") || { echo "缺少 YOLO 可执行文件: $YOLO_BINARY" >&2; exit 1; }
test -f $(printf '%q' "$YOLO_ENGINE") || { echo "缺少 YOLO engine: $YOLO_ENGINE" >&2; exit 1; }
test -f $(printf '%q' "$YOLO_NAMES") || { echo "缺少 YOLO 类别表: $YOLO_NAMES" >&2; exit 1; }

unlink $(printf '%q' "$YOLO_JSON") 2>/dev/null || true
pkill -TERM -f '[y]olo_trt' 2>/dev/null || true
sleep 0.5
pkill -KILL -f '[y]olo_trt' 2>/dev/null || true
setsid -f bash -c $(printf '%q' "cd $(printf '%q' "$REMOTE_ROOT/perception/yolo_trt") && exec $(printf '%q' "$YOLO_BINARY") --engine $(printf '%q' "$YOLO_ENGINE") --names $(printf '%q' "$YOLO_NAMES") --url $(printf '%q' "http://127.0.0.1:8765/frame.jpg") --image $(printf '%q' "$YOLO_IMAGE") --json $(printf '%q' "$YOLO_JSON") --frames 0 --interval $(printf '%q' "$YOLO_INTERVAL") --confidence $(printf '%q' "$YOLO_CONFIDENCE")") \
  >$(printf '%q' "$REMOTE_LOG_DIR/yolo.log") 2>&1 </dev/null
printf 'started YOLO-World\n'

pkill -TERM -f 'detection_http\.py' 2>/dev/null || true
sleep 0.2
setsid -f $(printf '%q' "$REMOTE_PYTHON_EXECUTABLE") $(printf '%q' "$REMOTE_ROOT/tools/detection_http.py") \
  --bind 0.0.0.0 --port $(printf '%q' "$DETECTION_PORT") --json $(printf '%q' "$YOLO_JSON") \
  >$(printf '%q' "$REMOTE_LOG_DIR/yolo_http.log") 2>&1 </dev/null
printf 'started detection JSON server on %s\n' $(printf '%q' "$DETECTION_PORT")
EOF

deadline=$((SECONDS + 20))
until curl -fsS --max-time 1 "$DETECTION_URL" 2>/dev/null | grep -Fq '"objects"'; do
  if (( SECONDS >= deadline )); then
    # shellcheck disable=SC2029
    ssh "$GO2_HOST" "tail -n 60 $(printf '%q' "$REMOTE_LOG_DIR/yolo.log") 2>/dev/null || true" >&2
    # shellcheck disable=SC2029
    ssh "$GO2_HOST" "tail -n 40 $(printf '%q' "$REMOTE_LOG_DIR/yolo_http.log") 2>/dev/null || true" >&2
    die "未收到 YOLO 检测 JSON: $DETECTION_URL"
  fi
  sleep 0.5
done
printf '检测 JSON 已就绪: %s\n' "$DETECTION_URL"

stop_local_pgids "Camera decoder" 'image_transport republish|image_transport/republish' quiet
stop_local_pgids "YOLO overlay" 'yolo_overlay_node( |$)' quiet
stop_local_pgids "Combined camera overlay" 'combined_camera_overlay\.py' quiet

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# shellcheck disable=SC1090
source "$LOCAL_WS/install/local_setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://${SCAN_LOCAL_DDS_CONFIG:-$HOME/.config/ros2/cyclonedds.xml}
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

mkdir -p "$LOCAL_LOG_DIR"
setsid -f bash -c \
  "cd $(printf '%q' "$LOCAL_WS") && exec $(printf '%q' "$OVERLAY_BINARY") --input-topic $(printf '%q' "$CAMERA_COMPRESSED_TOPIC") --output-topic $(printf '%q' "$CAMERA_TOPIC") --detections-url $(printf '%q' "$DETECTION_URL") --semantic-url $(printf '%q' "$SEMANTIC_URL") --stale-after $(printf '%q' "$DETECTION_STALE") --output-width $(printf '%q' "$OVERLAY_WIDTH")" \
  >"$LOCAL_LOG_DIR/combined_camera_overlay.log" 2>&1 </dev/null

if wait_local_topic "$CAMERA_TOPIC" 8; then
  printf 'SegFormer + YOLO 融合画面已启动: %s -> %s\n' "$CAMERA_COMPRESSED_TOPIC" "$CAMERA_TOPIC"
  printf '日志: %s/combined_camera_overlay.log\n' "$LOCAL_LOG_DIR"
else
  tail -n 60 "$LOCAL_LOG_DIR/combined_camera_overlay.log" >&2 2>/dev/null || true
  die "融合叠加节点未发布 $CAMERA_TOPIC"
fi

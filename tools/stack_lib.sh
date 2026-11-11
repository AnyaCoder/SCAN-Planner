#!/usr/bin/env bash
# Shared helpers for the Go2 stack launch scripts.

# Keep the Wi-Fi address in one place. Override GO2_HOST for an SSH alias and
# GO2_WIFI_HOST when the workstation uses the robot's direct hotspot instead.
GO2_WIFI_HOST=${GO2_WIFI_HOST:-192.168.77.18}
GO2_HOST=${GO2_HOST:-unitree@$GO2_WIFI_HOST}

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

stop_local_pgids() {
  local label=$1
  local pattern=$2
  local quiet=${3:-}
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
    [[ "$quiet" == "quiet" ]] || printf '  %s: 无运行进程\n' "$label"
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

# shellcheck disable=SC2034
front_camera_defaults() {
  CAMERA_ENABLE=${SCAN_CAMERA_ENABLE:-1}
  CAMERA_URL=${SCAN_CAMERA_URL:-http://$GO2_WIFI_HOST:8765/frame.jpg}
  CAMERA_FPS=${SCAN_CAMERA_FPS:-5}
  CAMERA_TOPIC=${SCAN_CAMERA_TOPIC:-/go2/front_camera/image_raw}
  CAMERA_COMPRESSED_TOPIC=$CAMERA_TOPIC/compressed
  case "$CAMERA_ENABLE" in
    1|true|on) CAMERA_ENABLED=1 ;;
    0|false|off) CAMERA_ENABLED=0 ;;
    *) CAMERA_ENABLED=0; return 1 ;;
  esac
}

front_camera_relay_running() {
  pgrep -f 'front_camera_relay\.py' >/dev/null 2>&1
}

front_camera_decoder_running() {
  pgrep -f 'image_transport republish' >/dev/null 2>&1
}

front_camera_calibration_running() {
  pgrep -f 'go2_camera_calibration.*front_camera_calibration\.launch\.py' >/dev/null 2>&1
}

front_camera_overlay_running() {
  pgrep -f 'yolo_overlay_node( |$)' >/dev/null 2>&1
}

front_camera_start() {
  local local_ws=$1
  local python=$2
  local tools_dir=$3
  local log_dir=$4
  mkdir -p "$log_dir"

  if ! front_camera_calibration_running; then
    if ros2 pkg prefix go2_camera_calibration >/dev/null 2>&1; then
      printf '启动前相机标定接口...\n'
      setsid -f bash -c \
        "cd $(printf '%q' "$local_ws") && exec ros2 launch go2_camera_calibration front_camera_calibration.launch.py" \
        >"$log_dir/camera_calibration.log" 2>&1 </dev/null
    else
      printf '警告: go2_camera_calibration 未构建，CameraInfo 和相机静态 TF 不会发布\n' >&2
    fi
  fi

  if ! front_camera_decoder_running && ! front_camera_overlay_running; then
    printf '启动相机 C++ 解码...\n'
    setsid -f bash -c \
      "cd $(printf '%q' "$local_ws") && exec ros2 run image_transport republish compressed in/compressed:=$(printf '%q' "$CAMERA_COMPRESSED_TOPIC") raw out:=$(printf '%q' "$CAMERA_TOPIC")" \
      >"$log_dir/camera_decoder.log" 2>&1 </dev/null
  fi

  if ! front_camera_relay_running; then
    printf '启动工作站前置相机...\n'
    setsid -f bash -c \
      "cd $(printf '%q' "$local_ws") && exec $(printf '%q' "$python") $(printf '%q' "$tools_dir/front_camera_relay.py") --url $(printf '%q' "$CAMERA_URL") --fps $(printf '%q' "$CAMERA_FPS") --topic $(printf '%q' "$CAMERA_COMPRESSED_TOPIC")" \
      >"$log_dir/camera.log" 2>&1 </dev/null
  fi
}

front_camera_stop() {
  stop_local_pgids "Front camera calibration" \
    'go2_camera_calibration.*front_camera_calibration\.launch\.py'
  stop_local_pgids "Front camera" 'front_camera_relay\.py'
  stop_local_pgids "Camera decoder" 'image_transport republish|image_transport/republish'
}

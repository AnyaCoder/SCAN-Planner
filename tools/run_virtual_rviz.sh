#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PYTHON_EXECUTABLE=${SCAN_PYTHON:-/usr/bin/python3}

if [[ -f /opt/ros/foxy/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/foxy/setup.bash
  set -u
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  printf 'rviz2 not found; source a ROS 2 Foxy environment first\n' >&2
  exit 1
fi

rviz2 -d "$SCRIPT_DIR/virtual_exploration.rviz" &
rviz_pid=$!
trap 'kill "$rviz_pid" 2>/dev/null || true' EXIT

"$PYTHON_EXECUTABLE" "$SCRIPT_DIR/pcd_virtual_exploration.py" \
  --rviz \
  --no-animation \
  "$@"

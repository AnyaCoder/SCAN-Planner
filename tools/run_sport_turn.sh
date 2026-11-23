#!/usr/bin/env bash
# Run the native Unitree Sport yaw test on Go2.
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REMOTE_SSH=${SCAN_REMOTE_SSH:-go2-wifi}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-/home/unitree/SCAN-Planner}
REMOTE_TOOLS_DIR=$REMOTE_ROOT/tools
REMOTE_SCRIPT=$REMOTE_TOOLS_DIR/sport_turn.py
REMOTE_GO2_ROOT=${SCAN_REMOTE_GO2_ROOT:-/home/unitree/go2}
REMOTE_DDS_RUNTIME=${SCAN_REMOTE_DDS_RUNTIME:-$REMOTE_GO2_ROOT/runtime/dds-0.10.2}
REMOTE_PYTHON=${SCAN_REMOTE_NATIVE_PYTHON:-$REMOTE_DDS_RUNTIME/venv/bin/python}

ssh -o BatchMode=yes "$REMOTE_SSH" "mkdir -p '$REMOTE_TOOLS_DIR'"
scp -q "$SCRIPT_DIR/sport_turn.py" "$REMOTE_SSH:$REMOTE_SCRIPT"

args=$(printf '%q ' "$@")
ssh -o BatchMode=yes "$REMOTE_SSH" "
  export PYTHONPATH='$REMOTE_GO2_ROOT/vendor/unitree_sdk2_python'\${PYTHONPATH:+:\$PYTHONPATH}
  export CYCLONEDDS_HOME='$REMOTE_DDS_RUNTIME/install'
  export LD_LIBRARY_PATH='$REMOTE_DDS_RUNTIME/install/lib'\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}
  exec '$REMOTE_PYTHON' '$REMOTE_SCRIPT' --execute $args
"

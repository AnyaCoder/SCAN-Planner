#!/usr/bin/env bash
# Run on Go2 after the MID-360 driver is publishing /livox/lidar.
set -Eeo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REMOTE_HOME=${SCAN_REMOTE_HOME:-/home/unitree}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-$REMOTE_HOME/SCAN-Planner}
REMOTE_GRAPH_WS=${SCAN_REMOTE_GRAPH_WS:-/unitree/module/graph_pid_ws}
REMOTE_FASTLIO_WS=${SCAN_REMOTE_FASTLIO_WS:-$REMOTE_HOME/fastlio_ws}
REMOTE_AUTONOMY_WS=${SCAN_REMOTE_AUTONOMY_WS:-$REMOTE_HOME/autonomy_stack_go2}

# Keep the same workspace and library ordering as start_scan_stack.sh. The Go2
# Foxy RMW is used with the workspace's CycloneDDS 0.10.2 in /usr/local/lib.
source /opt/ros/foxy/setup.bash
source "$REMOTE_GRAPH_WS/install/local_setup.bash"
source "$REMOTE_FASTLIO_WS/install/local_setup.bash"
source "$REMOTE_AUTONOMY_WS/install/local_setup.bash"
source "$REMOTE_ROOT/install/local_setup.bash"

export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$REMOTE_HOME/.config/ros2/cyclonedds.xml
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=0

exec /usr/bin/python3 "$SCRIPT_DIR/capture_mid360_topdown.py" "$@"

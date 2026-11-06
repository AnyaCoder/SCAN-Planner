#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=${SCAN_LOCAL_WS:-$(cd -- "$SCRIPT_DIR/.." && pwd)}
GO2_HOST=${GO2_HOST:-go2-wifi}
REMOTE_HOME=${SCAN_REMOTE_HOME:-/home/unitree}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-$REMOTE_HOME/SCAN-Planner}
REMOTE_GRAPH_WS=${SCAN_REMOTE_GRAPH_WS:-/unitree/module/graph_pid_ws}
REMOTE_FASTLIO_WS=${SCAN_REMOTE_FASTLIO_WS:-$REMOTE_HOME/fastlio_ws}
REMOTE_DDS_CONFIG=${SCAN_REMOTE_DDS_CONFIG:-$REMOTE_HOME/.config/ros2/cyclonedds.xml}
REMOTE_MAP=${REMOTE_MAP:-$REMOTE_ROOT/maps/go2_scan.pcd}
WAYPOINTS=${1:-$WS/maps/clicked_waypoints.yaml}
BUNDLE=${2:-$WS/maps/bundles/$(date +%Y%m%d-%H%M%S)}
BRIDGE_CONFIG=$WS/src/planner/lio_bridge/config/fastlio_bridge.yaml
PYTHON_EXECUTABLE=${SCAN_PYTHON:-/usr/bin/python3}

[[ -f "$WAYPOINTS" ]] || {
  printf '错误: 找不到路径点文件: %s\n' "$WAYPOINTS" >&2
  exit 1
}
[[ -f "$BRIDGE_CONFIG" ]] || {
  printf '错误: 找不到 bridge 配置: %s\n' "$BRIDGE_CONFIG" >&2
  exit 1
}

# shellcheck disable=SC2087
ssh -o ConnectTimeout=5 "$GO2_HOST" bash -s <<REMOTE
set +u
source /opt/ros/foxy/setup.bash
source "$REMOTE_GRAPH_WS/install/local_setup.bash"
source "$REMOTE_FASTLIO_WS/install/local_setup.bash"
source "$REMOTE_ROOT/install/local_setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$REMOTE_DDS_CONFIG
export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}
if ! pgrep -f 'fastlio_mapping|fast_lio mapping\.launch\.py' >/dev/null 2>&1; then
  printf '错误: FAST-LIO 未运行，无法保存与路径点同 world 的地图\n' >&2
  exit 1
fi
mkdir -p "$REMOTE_ROOT/maps"
timeout 60 ros2 service call /map_save std_srvs/srv/Trigger {}
test -s "$REMOTE_MAP"
REMOTE

mkdir -p "$BUNDLE"
scp -q "$GO2_HOST:$REMOTE_MAP" "$BUNDLE/map_camera_init.pcd"
"$PYTHON_EXECUTABLE" "$SCRIPT_DIR/align_scan_pcd.py" \
  "$BUNDLE/map_camera_init.pcd" \
  "$BUNDLE/map_world.pcd" \
  --bridge-config "$BRIDGE_CONFIG"
cp "$WAYPOINTS" "$BUNDLE/clicked_waypoints.yaml"

"$PYTHON_EXECUTABLE" - "$BUNDLE" "$REMOTE_MAP" "$BRIDGE_CONFIG" <<'PY'
import hashlib
import sys
from pathlib import Path

import yaml

bundle = Path(sys.argv[1])
remote_map = sys.argv[2]
bridge_config = Path(sys.argv[3])
params = yaml.safe_load(bridge_config.read_text())["lio_bridge"]["ros__parameters"]

def md5(path):
    return hashlib.md5(Path(path).read_bytes()).hexdigest()

metadata = {
    "map_frame": "world",
    "local_frame": "fastlio_world",
    "prior_map": "map_world.pcd",
    "raw_fastlio_map": "map_camera_init.pcd",
    "waypoints": "clicked_waypoints.yaml",
    "remote_map": remote_map,
    "world_alignment": [
        params["world_alignment_x"],
        params["world_alignment_y"],
        params["world_alignment_z"],
        params["world_alignment_w"],
    ],
    "files": {
        name: {"md5": md5(bundle / name), "bytes": (bundle / name).stat().st_size}
        for name in ("map_world.pcd", "map_camera_init.pcd", "clicked_waypoints.yaml")
    },
}
(bundle / "bundle.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
print(f"bundle: {bundle}")
print(f"prior map: {bundle / 'map_world.pcd'}")
print(f"waypoints: {bundle / 'clicked_waypoints.yaml'}")
PY

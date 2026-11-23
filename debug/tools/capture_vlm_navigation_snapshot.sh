#!/usr/bin/env bash
# Snapshot the current camera and render the in-memory five-second point cloud.
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
LOCAL_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
REMOTE_SSH=${SCAN_REMOTE_SSH:-go2-wifi}
REMOTE_ROOT=${SCAN_REMOTE_ROOT:-/home/unitree/SCAN-Planner}
GO2_WIFI_HOST=${GO2_WIFI_HOST:-192.168.77.2}
CAMERA_URL=${SCAN_CAMERA_URL:-http://$GO2_WIFI_HOST:8765/frame.jpg}
OUTPUT_DIR=$LOCAL_ROOT/debug/pointcloud_views
REMOTE_OUTPUT_DIR=$REMOTE_ROOT/debug/pointcloud_views
METADATA_FILE=$OUTPUT_DIR/vlm_navigation_current.json

mkdir -p "$OUTPUT_DIR"

snapshot_sequence=$(
  python3 - "$METADATA_FILE" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
try:
    sequence = int(json.loads(path.read_text())["snapshot_sequence"])
except (OSError, ValueError, KeyError, json.JSONDecodeError):
    sequence = 0
print(sequence + 1)
PY
)

work=$(mktemp -d)
camera_file=$OUTPUT_DIR/front_camera_current.jpg
camera_tmp=$OUTPUT_DIR/.front_camera_current.$$.jpg
trap 'rm -f "$camera_tmp"; rm -rf "$work"' EXIT

curl -fsS --max-time 4 "$CAMERA_URL" -o "$camera_tmp"
[[ -s "$camera_tmp" ]] || {
  printf 'camera snapshot is empty: %s\n' "$CAMERA_URL" >&2
  exit 1
}
mv -f "$camera_tmp" "$camera_file"

remote_live=$REMOTE_OUTPUT_DIR/mid360_topdown_live.png
remote_json=${remote_live%.png}.json
remote_trigger=$REMOTE_OUTPUT_DIR/mid360_topdown_request
remote_ready=$REMOTE_OUTPUT_DIR/mid360_topdown_ready.json

trigger_render() {
  ssh -o BatchMode=yes "$REMOTE_SSH" "
    if ! pgrep -f '[c]apture_mid360_topdown\.py --rolling' \
      >/dev/null 2>&1 || [[ ! -s '$remote_ready' ]]; then
      exit 9
    fi
    rm -f '$remote_json' '$remote_trigger'
    touch '$remote_trigger'
    python3 - '$remote_json' <<'PY'
import os
import sys
import time

deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    if os.path.isfile(sys.argv[1]) and os.path.getsize(sys.argv[1]) > 0:
        raise SystemExit(0)
    time.sleep(0.05)
raise SystemExit('timed out waiting for on-demand top-down render')
PY
  "
}

if ! trigger_render; then
  "$SCRIPT_DIR/start_rolling_topdown.sh"
  trigger_render
fi

scp -q "$REMOTE_SSH:$remote_live" "$work/" &
map_pid=$!
scp -q "$REMOTE_SSH:$remote_json" "$work/" &
metadata_pid=$!
wait "$map_pid"
wait "$metadata_pid"

python3 - <<'PY' "$work/mid360_topdown_live.json"
import json
import sys
import time

with open(sys.argv[1], encoding="utf-8") as stream:
    metadata = json.load(stream)
age = time.time() - float(metadata["generated_at_unix"])
if age > 4.0:
    raise SystemExit("rolling top-down is stale: %.2f s" % age)
if not metadata.get("rolling_ready"):
    raise SystemExit("rolling top-down is not ready")
PY

python3 "$SCRIPT_DIR/compose_vlm_navigation_snapshot.py" \
  --annotated-map "$work/mid360_topdown_live.png" \
  --camera "$camera_file" \
  --map-metadata "$work/mid360_topdown_live.json" \
  --clean-map-output "$OUTPUT_DIR/mid360_topdown_5s.png" \
  --metadata-output "$OUTPUT_DIR/vlm_navigation_current.json" \
  --snapshot-sequence "$snapshot_sequence"

cp "$work/mid360_topdown_live.json" "$OUTPUT_DIR/mid360_topdown_5s.json"

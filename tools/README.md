Calibration, point-cloud capture, and VLM snapshot debugging helpers live under
[`debug/tools/`](../debug/tools/). This directory contains the runtime entry
points for the stack.

# Native in-place turn

`run_sport_turn.sh` drives the Go2 yaw through its native Unitree Sport API. It
keeps API `1008` active at 100 Hz, uses `/sportmodestate` IMU yaw as feedback,
then sends zero `Move` and API `1003` `StopMove`. It does not publish a SCAN
goal or use `/cmd_vel`.

```bash
# Dry run: print the command without moving the robot.
python3 tools/sport_turn.py --direction right --angle-deg 15

# Run on Go2 through the hotspot SSH alias.
./tools/run_sport_turn.sh --direction right --angle-deg 15
./tools/run_sport_turn.sh --direction left --angle-deg 72
```

Rates below `0.5 rad/s` are rejected because they can fall inside the Go2
Sport yaw deadband.

# Relocalization replay

`relocalization_replay.sh` runs the validated
FAST-LIO -> `map_relocalizer` -> SCAN chain:

```bash
./tools/relocalization_replay.sh start
./tools/relocalization_replay.sh replay
./tools/relocalization_replay.sh stop
./tools/relocalization_replay.sh status
```

`start` accepts a bundle directory or `map_world.pcd`. It waits for
`2D Pose Estimate` and ICP convergence before starting SCAN. `replay` only
restarts SCAN when relocalization is already converged.

# FAST-LIO2 (ROS 2 Foxy)

`install_fastlio_foxy.sh` checks out the pinned FAST-LIO2 revision, applies
the Foxy, TF-remap, and gravity-alignment patches, and builds `fast_lio`:

```bash
./tools/install_fastlio_foxy.sh ~/fastlio_ws
```

The Go2 deployment and runtime sequence are documented in the repository root
`必看.md`.

# World alignment calibration

`calibrate_world_alignment.py` fits the local ground plane from
`/grid_map/occupancy` and prints the quaternion for
`src/planner/lio_bridge/config/fastlio_bridge.yaml`:

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.config/ros2/cyclonedds.xml
python3 debug/tools/calibrate_world_alignment.py
```

Run it on a stationary robot with a fresh map. The default `1.5 m` radius and
lowest `12%` height band isolate the floor around the robot.

# Wi-Fi TCP state bridge

`scan_wifi_bridge.py` keeps ROS/DDS local to each machine and sends only the
SCAN visualization graph across the Go2 Wi-Fi link:

- Go2 DDS: `eth0`, including `--server`
- workstation DDS: `lo`, including `--client` and RViz
- cross-machine: TCP `9000`

On Go2:

```bash
python3 /home/unitree/SCAN-Planner/tools/scan_wifi_bridge.py \
  --server --bind 0.0.0.0 --port 9000
```

On the workstation:

```bash
python3 tools/scan_wifi_bridge.py \
  --client --host 192.168.77.2 --port 9000
```

The bridge keeps only the newest sample per topic, rejects stale backlogs, and
compensates the timestamp offset between the two clocks in both directions.
The TCP server accepts multiple clients concurrently. Each client has its own
latest-value queue, so a slow client drops old frames without blocking another
client. A non-latched topic is serialized only while at least one connected
client subscribes to it, so the bridge does not spend Go2 CPU building frames
that no one will receive.

Each client can limit the uplink topics it receives:

```bash
# Full SCAN visualization, the default
python3 tools/scan_wifi_bridge.py \
  --client --host 192.168.77.2 --port 9000

# Point cloud and pose only
python3 tools/scan_wifi_bridge.py \
  --client --host 192.168.77.2 --port 9000 \
  --topics /LIO/clouds_lidar,/LIO/odom_vehicle,/tf,/tf_static

# Control/downlink only; receive no uplink topics
python3 tools/scan_wifi_bridge.py \
  --client --host 192.168.77.2 --port 9000 --topics none
```

The filter is negotiated during the clock handshake. Topics not requested by
any connected client are skipped before serialization. The bridge protocol
version is bumped whenever this handshake changes; all machines must be
updated together.
Global compression is off by default. `/grid_map/occupancy_inflate` is always
compressed per-topic and limited to 1 Hz because a raw frame is about 3.9 MB;
use global `--compress` only on a severely bandwidth-limited link.
`start_scan_stack.sh` starts and stops both ends automatically.

The workstation pulls the Go2 front camera directly from
`http://192.168.77.2:8765/frame.jpg` over Wi-Fi, without adding a ROS node on
the robot or passing camera frames through the state bridge. A separate
`front_camera_relay.py` process publishes `CompressedImage`, then the C++
`image_transport republish` node decodes it to
`/go2/front_camera/image_raw` for the RViz `Front Camera` display.
The workstation needs `ros-foxy-compressed-image-transport`.
Both the front camera and VLM overlay start by default with the full stack. Set
`SCAN_CAMERA_ENABLE=0` to disable the camera, or `SCAN_VLM_ENABLE=0` to keep the
camera but skip VLM navigation requests.
The Unitree source is 1920x1080; `front_camera_bridge.py` downsamples it to
1280x720 before JPEG encoding. The default is 5 Hz, about 0.45 MB/s; raise it
with `SCAN_CAMERA_FPS=15` if needed. Disable the camera with
`SCAN_CAMERA_ENABLE=0`. YOLO, SegFormer, and the depth-color camera overlay are
disabled by default and can be enabled explicitly with their `SCAN_*_ENABLE`
environment variables.

## YOLO overlay

The detector runs on Go2 and writes a small JSON file. The workstation polls
that JSON and draws the boxes while decoding the existing camera stream, so no
second video stream is sent over Wi-Fi. The annotated frames replace the
`image_transport republish` decoder and still use
`/go2/front_camera/image_raw`, which means the existing RViz `Front Camera`
display needs no change. The combined overlay keeps the calibrated
`1280x720` camera resolution by default so image pixels stay in the same
coordinate system as `CameraInfo`, SegFormer masks, and camera projections.
Set `SCAN_OVERLAY_WIDTH` only when a smaller display is explicitly desired.

For pixel-to-world validation, start the VLM shadow node with
`./tools/start_vlm_pixel_shadow.sh`. The VLM first returns one normalized
camera-floor point such as `point: 0.62 0.41`. It requests the top-down fallback
only when the camera is blocked by a nearby wall or cannot see floor.
The fallback request receives only the 10 m dual-LiDAR top-down map and returns
one signed in-place turn such as `turn: -72`, relative to the orange dog
heading. Positive angles turn left and negative angles turn right.
The launcher now defaults to the multimodal
`unsloth/Qwen3.5-27B-GGUF` llama.cpp server behind an SSH tunnel to
`AnyaCoder@10.126.62.90:127.0.0.1:8001`. Set
`SCAN_VLM_API_BACKEND=remote` and provide `SCAN_VLM_API_URL`,
`SCAN_VLM_MODEL`, plus `VLM_API_KEY` in the Go2 `~/.env` to use another
OpenAI-compatible multimodal endpoint.
The local ground plane is estimated from the LiDAR cloud rather than assuming
`z=0`. After locking, no further model requests are made: the fixed world
point is projected back through the calibrated fisheye camera at the camera
rate, so its image marker follows naturally as Go2 moves. The pin and ray are
published on `/vlm_pixel_shadow`, while the live debug image is published on
`/go2/front_camera/vlm_point_overlay`. The prompt targets a visible mid-range
ground point rather than the farthest visible ground. Targets farther than
`SCAN_VLM_PIXEL_MAX_DISTANCE` (default `4.5` m from robot center) are rejected
and never locked.

Navigation is approval-gated. A locked point does not publish anything to
`/move_base_simple/goal` until `批准并发送到 SCAN` is clicked in the
`VLM Approval` RViz panel. Approval publishes exactly one goal to SCAN;
an approved top-down turn instead calls the native Unitree Sport yaw API at
100 Hz and leaves `/move_base_simple/goal` untouched. When the turn finishes,
the old target and markers are cleared, and the camera is observed again before
the next VLM decision.
`让 VLM 重选` clears the locked point, marker, and approval state, captures a
fresh text-free front-camera image and 10 m point-cloud image with the current
`/LIO/odom_vehicle` pose, and sends the two images separately in camera-then-map
order. The long-running collector holds only the latest five seconds of MID-360
and Go2 native LiDAR frames in memory. It writes no repeated images; every VLM
query or reselect triggers one on-demand render. Both LiDARs are transformed to
a shared world frame using each frame's own odometry stamp, fused there, and then
transformed once into the current Go2 IMU frame. Floor returns are gray,
obstacles are green/cyan, and returns above 1.20 m are removed before rendering.
The model returns no route list or candidate enumeration. For normal movement
the node preserves the model-selected normalized camera point as the world
target and only validates that gray floor exists within 0.20 m and that the
center is not a green obstacle. SCAN-Planner remains responsible for the actual
path and collision checks.
SCAN still performs its normal planning, collision checks, and emergency stop.
Set `SCAN_VLM_DRIVE=0` for display-only validation.

The current general-purpose VLM was not trained on this rendered top-down map.
Camera-point projection is the primary path. Top-down angles remain
approval-gated because their direction and magnitude can be inaccurate.

Build the workstation node once:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select scan_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Start or stop the overlay without restarting LIO or SCAN:

```bash
./tools/start_yolo_overlay.sh
./tools/stop_yolo_overlay.sh
```

To start it together with the full stack:

```bash
SCAN_YOLO_ENABLE=1 ./tools/start_scan_stack.sh --restart
```

Go2 defaults are `perception/yolo_trt/build/yolo_trt` with the fixed-vocabulary
YOLO-World engine `perception/engines/yolov8s_worldv2_indoor_fp16.engine` and
the existing camera endpoint `http://127.0.0.1:8765/frame.jpg`. The startup
script syncs `perception/yolo_world/classes.txt` to Go2 and passes it through
`--names`, so the labels match the exported vocabulary. Detection JSON is
exposed on port `8767`;
port `8766` is already used by `live_robot_state.py`. The detector runs at
5 Hz (`SCAN_YOLO_INTERVAL=0.2`), while annotated camera frames stay at the
normal camera rate. On the current workstation the C++ overlay uses about 50%
of one CPU core with RViz subscribed; the plain `image_transport` decoder uses
about 27%, so the overlay adds roughly 8 percentage points. Camera output is
set to 5 Hz to match the detector.

Useful overrides are `SCAN_YOLO_ENGINE`, `SCAN_YOLO_NAMES`,
`SCAN_YOLO_CLASSES_SOURCE`, `SCAN_YOLO_CONFIDENCE`, `SCAN_YOLO_INTERVAL`,
`SCAN_YOLO_PORT`, and `SCAN_YOLO_STALE`. Set `SCAN_YOLO_ENGINE` to
`perception/engines/yolov8n_fp16.engine` to fall back to the old COCO detector;
omit `SCAN_YOLO_CLASSES_SOURCE` only when the remote `SCAN_YOLO_NAMES` file is
already present.

The open vocabulary lives in `perception/yolo_world/classes.txt`. Export and
engine-build commands are in `perception/README.md`.

Logs:

- `/tmp/scan_planner_logs/yolo.log`
- `/tmp/scan_planner_logs/yolo_http.log`
- `/tmp/scan_planner_local_logs/combined_camera_overlay.log`

`restart_rviz.sh` reopens only the workstation RViz window when the Go2 stack
is still running. It validates `/tf`, `/LIO/clouds_lidar`, and `world -> body`,
and restarts only the local TCP client if the visualization data stream is
stale. It never stops Go2 algorithms or the speed bridge.

# Click waypoint editor

`start_scan_stack.sh` starts `clicked_waypoint_recorder.py` automatically in
`navi_mode=1` in a separate terminal. It records RViz `Publish Point` clicks as
numbered waypoints and writes `maps/clicked_waypoints.yaml` by default.
Nothing is saved until you press `s` or `q` in that terminal. Its log is
`/tmp/scan_planner_local_logs/clicked_waypoints.log`.
The automatic recorder publishes every click to `/move_base_simple/goal`, so
the dog drives to that point as it is recorded. Set
`SCAN_CLICKED_WAYPOINTS_DRIVE=0` to record without moving.

To run the recorder manually instead:

```bash
ros2 run scan_planner clicked_waypoint_recorder.py \
  --output maps/clicked_waypoints.yaml \
  --z 0.4
```

In the normal `default_real.rviz` view use `Publish Point` with fixed frame
`world`. The `Clicked Waypoints` MarkerArray layer shows the ordered points and
numbers.
Terminal controls:

- `u`: undo the last point
- `c`: clear all
- `s`: save
- `l`: list
- `q`: save and quit

To make the dog drive to each clicked point while recording, add `--drive`.
This publishes each point to `/move_base_simple/goal`, so it only works with
SCAN `navi_mode=1` and the speed bridge enabled:

```bash
ros2 run scan_planner clicked_waypoint_recorder.py \
  --drive \
  --output maps/clicked_waypoints.yaml \
  --z 0.4
```

Without `--drive`, clicks are recording-only. With `--drive`, undo removes the
saved point but does not cancel a goal the dog is already executing.

The output is directly usable by `navi_mode=2`:

```yaml
scan_planner_node:
  ros__parameters:
    fsm.navi_mode: 2
    fsm.waypoints: [x0, y0, z0, x1, y1, z1]
```

Copy the YAML to Go2 and start waypoint navigation with:

```bash
SCAN_NAVI_MODE=2 \
SCAN_KEYPOINTS_FILE=/home/unitree/SCAN-Planner/maps/clicked_waypoints.yaml \
./tools/start_scan_stack.sh --restart
```

If the saved PCD is in another frame, set it to `world` or transform it first;
the recorder ignores clicks whose frame is not `world` or `map`.

# Dense PCD save

`fast_lio_dense_map_save.patch` appends every registered LiDAR frame to a dense
save buffer, then voxel-downsamples once at save time. The path and leaf size
are configured in `mid360_fastlio.yaml`:

```yaml
map_file_path: "/home/unitree/SCAN-Planner/maps/go2_scan.pcd"
pcd_save:
  pcd_save_en: true
  voxel_size: 0.05
```

To save the dense map together with the path points while the stack is still
running, use `save_relocalization_bundle.sh` as described above.

Set `SCAN_SAVE_MAP_ON_STOP=1` to save the raw dense map before
`stop_scan_stack.sh` stops FAST-LIO2. Saving is disabled by default to avoid
writing a large PCD on every restart. Longer scans use more RAM until the final
voxel filter runs.

Reference DDS configurations:

```text
tools/cyclonedds_go2_eth0.xml
tools/cyclonedds_workstation_lo.xml
```

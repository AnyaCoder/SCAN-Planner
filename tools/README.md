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
python3 tools/calibrate_world_alignment.py
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
  --client --host 192.168.76.1 --port 9000
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
  --client --host 192.168.76.1 --port 9000

# Point cloud and pose only
python3 tools/scan_wifi_bridge.py \
  --client --host 192.168.76.1 --port 9000 \
  --topics /LIO/clouds_lidar,/LIO/odom_vehicle,/tf,/tf_static

# Control/downlink only; receive no uplink topics
python3 tools/scan_wifi_bridge.py \
  --client --host 192.168.76.1 --port 9000 --topics none
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
`http://192.168.76.1:8765/frame.jpg` over Wi-Fi, without adding a ROS node on
the robot or passing camera frames through the state bridge. A separate
`front_camera_relay.py` process publishes `CompressedImage`, then the C++
`image_transport republish` node decodes it to
`/go2/front_camera/image_raw` for the RViz `Front Camera` display.
The workstation needs `ros-foxy-compressed-image-transport`.
The Unitree source is 1920x1080; `front_camera_bridge.py` downsamples it to
1280x720 before JPEG encoding. At the default 15 Hz target this is about
1.4 MB/s; the current workstation observes 11-13 Hz end to end. Disable it
with `SCAN_CAMERA_ENABLE=0`; lower the bandwidth with `SCAN_CAMERA_FPS=5`.

`restart_rviz.sh` reopens only the workstation RViz window when the Go2 stack
is still running. It validates `/tf`, `/LIO/clouds_lidar`, and `world -> body`,
and restarts only the local TCP client if the visualization data stream is
stale. It never stops Go2 algorithms or the speed bridge.

# Click waypoint editor

`start_scan_stack.sh` starts `clicked_waypoint_recorder.py` automatically in
`navi_mode=1` in a separate terminal. It records RViz `Publish Point` clicks as
numbered waypoints and writes `foxy_ws/maps/clicked_waypoints.yaml` by default.
Nothing is saved until you press `s` or `q` in that terminal. Its log is
`/tmp/scan_planner_local_logs/clicked_waypoints.log`.
The automatic recorder publishes every click to `/move_base_simple/goal`, so
the dog drives to that point as it is recorded. Set
`SCAN_CLICKED_WAYPOINTS_DRIVE=0` to record without moving.

To run the recorder manually instead:

```bash
ros2 run scan_planner clicked_waypoint_recorder.py \
  --output ~/SCAN-Planner/foxy_ws/maps/clicked_waypoints.yaml \
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
  --output ~/SCAN-Planner/foxy_ws/maps/clicked_waypoints.yaml \
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

`stop_scan_stack.sh` saves the raw dense map automatically before stopping
FAST-LIO2.
Longer scans use more RAM until the final voxel filter runs.

Reference DDS configurations:

```text
tools/cyclonedds_go2_eth0.xml
tools/cyclonedds_workstation_lo.xml
```

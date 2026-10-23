# Keypoint recorder (ROS 2)

Build and source the workspace, then run:

```bash
ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle --output keypoints.yaml
```

The generated file is a ROS 2 parameter file containing the flat
`fsm.waypoints: [x0, y0, z0, ...]` array. Pass it to `run.launch.py` with
`keypoints_file:=/absolute/path/to/keypoints.yaml` when `navi_mode:=2`.

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
Global compression is off by default. `/grid_map/occupancy_inflate` is always
compressed per-topic and limited to 1 Hz because a raw frame is about 3.9 MB;
use global `--compress` only on a severely bandwidth-limited link.
`start_scan_stack.sh` starts and stops both ends automatically.

`restart_rviz.sh` reopens only the workstation RViz window when the Go2 stack
is still running. It validates `/tf`, `/LIO/clouds_lidar`, and `world -> body`,
and restarts only the local TCP client if the visualization data stream is
stale. It never stops Go2 algorithms or the speed bridge.

Reference DDS configurations:

```text
tools/cyclonedds_go2_eth0.xml
tools/cyclonedds_workstation_lo.xml
```

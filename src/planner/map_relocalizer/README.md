# FAST-LIO PCD relocalization

This package keeps FAST-LIO as the local odometry source and relocalizes each
new `fastlio_world` session against a saved, world-aligned PCD map.

```text
FAST-LIO
  -> lio_bridge (output frame: fastlio_world)
       -> /LIO/odom_vehicle
       -> /LIO/odom_imu
       -> /LIO/clouds_lidar

map_relocalizer
  - loads prior world-aligned PCD
  - takes /initialpose as the initial base pose in `world`
  - runs ICP against the prior map
  - publishes world -> fastlio_world TF
  - publishes /LIO/relocalized/odom_vehicle
  - publishes /LIO/relocalized/odom_imu

SCAN-Planner
  -> use_relocalized_odom:=true
  -> reuses waypoints recorded in `world`
```

## Build

```bash
cd "$(git rev-parse --show-toplevel)"
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select map_relocalizer scan_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Save a reusable bundle

Run this while the recording FAST-LIO/SCAN session is still alive:

```bash
cd "$(git rev-parse --show-toplevel)"
./tools/save_relocalization_bundle.sh \
  maps/clicked_waypoints.yaml
```

It saves `/map_save`, copies the PCD, applies `world_alignment` to produce
`map_world.pcd`, copies the waypoint YAML, and writes `bundle.yaml`.

## Start a relocalization session

1. Start Livox and FAST-LIO normally.
2. Start `lio_bridge` with the local frame name:

```bash
ros2 launch lio_bridge fastlio_bridge.launch.py \
  output_frame_id:=fastlio_world
```

3. Start the relocalizer with the aligned prior map:

```bash
ros2 launch map_relocalizer map_relocalizer.launch.py \
  prior_map_file:=/absolute/path/to/bundle/map_world.pcd
```

4. In the relocalization RViz window, use `2D Pose Estimate` to place the dog
   roughly in the prior map, then wait for:

```text
/relocalization/status: converged fitness=...
```

Do not start SCAN before convergence.

5. Start SCAN with the relocalized odometry:

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  use_relocalized_odom:=true \
  navi_mode:=2 \
  keypoints_file:=/absolute/path/to/bundle/clicked_waypoints.yaml \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  publish_robot_state:=false
```

The waypoint file must be recorded in the same `world` as the prior map.

## Limitations

- The initial pose must be reasonably close; this is local ICP, not global
  place recognition.
- ICP currently runs only until the first convergence. Re-send `2D Pose
  Estimate` to reset it.
- The prior PCD must be world-aligned. Do not pass the raw FAST-LIO
  `camera_init` PCD directly.

# Debug workspace

This directory holds diagnostic helpers and generated artifacts that are not
part of the normal Go2 runtime entry points.

```text
debug/
  tools/             calibration, point-cloud capture, and VLM snapshot helpers
  pointcloud_views/  on-demand camera/top-down snapshots (ignored by Git)
  calibration/       calibration reports and overlays (ignored by Git)
  maps/              sample/debug point-cloud maps
```

Normal stack startup, stopping, control, networking, and ROS services remain in
`tools/`. Do not move those runtime entry points here.

The VLM snapshot flow renders the live five-second point-cloud window on demand.
No image sequence is written continuously.

Typical commands:

```bash
./debug/tools/capture_vlm_navigation_snapshot.sh
./debug/tools/start_rolling_topdown.sh
python3 debug/tools/calibrate_world_alignment.py
python3 debug/tools/calibrate_l1_to_mid360.py
```

## Sport yaw / right-stick emulation

`sport_turn.py` uses the native Unitree GO2 SDK to send API `1008` with
`x=0, y=0, z=<yaw rate>`. It keeps the command active at 50 Hz and uses
`/sportmodestate` IMU yaw as feedback, then sends zero `Move` and API `1003`
`StopMove`. This bypasses the ROS bridge while preserving the same Sport
channel used by the physical controller's right stick.

```bash
# Dry run: print the command but do not move the robot.
python3 debug/tools/sport_turn.py --direction right --angle-deg 15

# Run on Go2: default is a 15 degree right turn at 0.8 rad/s.
./debug/tools/run_sport_turn.sh

# Other rotations.
./debug/tools/run_sport_turn.sh --direction left --angle-deg 30
./debug/tools/run_sport_turn.sh --direction right --duration 0.6
```

Rates below `0.5 rad/s` are rejected because they can fall inside the GO2
Sport yaw deadband. The tool keeps other upper-layer ROS services optional;
only the native robot Sport service and `eth0` network are required.

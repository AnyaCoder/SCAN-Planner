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

# Go2 front camera calibration

This package is the canonical location for the calibrated Go2 front fisheye
camera. It publishes the values as standard ROS interfaces instead of requiring
individual nodes to hard-code camera constants.

## Calibrated interfaces

- Intrinsics and fisheye distortion:
  `/go2/front_camera/camera_info`
- Extrinsic transform:
  `body -> go2_front_camera_optical_frame`

The active stream used by SCAN-Planner is 1280x720. The original 30-image
checkerboard calibration was performed at 1920x1080 and is retained in
`camera.native`; both intrinsic sets describe the same camera and the distortion
coefficients are shared.

The extrinsic convention is:

```text
p_body = T_body_camera * p_camera_optical
```

The camera optical axes are `x` right, `y` down and `z` forward. `body` is the
frame used by `/LIO/clouds_lidar`. The FAST-LIO LiDAR-IMU extrinsic is a
different transform and must not be substituted for this camera extrinsic.

The direct visual calibration rotation is retained as
`extrinsic.calibrated_quaternion_xyzw`. The published orientation also applies
the L1 LiDAR mounting pitch of `-15.1 deg`, so the tilted
`/LIO/clouds_lidar` cloud is projected with a level camera optical axis. The
fitted ground-plane normal is within `0.59 deg` of horizontal. The
`0.046825 m` body-z camera offset is a physical translation, not an image-space
shift.

Distance origins must remain separate:

- VLM image rays start at `go2_front_camera_optical_frame`.
- RViz target and navigation ranges are measured from the robot-center `base`
  frame.
- LiDAR points are measurements in `body`; after projection they are not
  independent camera range measurements, so the camera ray-ground
  intersection is authoritative for the target position.

## Launch

```bash
source install/setup.bash
ros2 launch go2_camera_calibration front_camera_calibration.launch.py
```

Check the interfaces in another terminal:

```bash
ros2 run tf2_ros tf2_echo body go2_front_camera_optical_frame
ros2 topic echo --once /go2/front_camera/camera_info \
  --qos-durability transient_local
```

`tools/front_camera_start` in `tools/stack_lib.sh` launches this package with
the SCAN-Planner front-camera relay, so `tools/start_scan_stack.sh` does not
need a separate calibration command.

## Result quality

The intrinsic calibration used 30 checkerboard images and has RMS reprojection
error 0.4369 px. The camera-LiDAR calibration used five static image/point-cloud
scenes and direct normalized-information-density registration. Using only the
first four scenes versus all five changes the extrinsic by 0.17 mm and
0.0092 degrees.

The projection validation for all five scenes is installed as
`validation/front_camera_lidar_projection.jpg`.

## Important image-resolution rule

`camera_info` describes the 1280x720 source images. If a downstream node emits a
resized image, it must either scale `K` and `P` accordingly or publish a
separate `CameraInfo`. Do not use the 1920x1080 matrix with 1280x720 images.

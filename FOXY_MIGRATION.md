# ROS 2 Foxy Migration

This workspace is based on the upstream `ros2-community` branch and has been
backported to Ubuntu 20.04 / ROS 2 Foxy for the Go2 onboard computer.

## Verified

- Ubuntu 20.04, ROS 2 Foxy, GCC 9.4
- Full workspace build: 13 packages finished
- Headless SCAN-Planner simulation starts all required nodes
- A goal published on `/move_base_simple/goal` produces `/planning/bspline`
  and drives the kinematic simulator through `/quad_0/cmd_vel`

## Compatibility Changes

- Subscription callbacks now take `ConstSharedPtr` by value. Foxy does not
  support the Humble-style `const ConstSharedPtr &` callback binding used by
  the community branch.
- `plan_env` exports its OpenCV and PCL CMake dependencies. Without this,
  downstream packages fail on PCL 1.10 because the imported `FLANN::FLANN`
  target is not defined in the consumer project.
- `tf2_geometry_msgs/tf2_geometry_msgs.hpp` and `tf2/utils.hpp` have Foxy
  fallbacks to the corresponding `.h` headers. `__has_include` keeps the code
  compatible with Humble.

## Build

```bash
cd ~/SCAN-Planner/foxy_ws
source /opt/ros/foxy/setup.bash
sudo apt install libarmadillo-dev libeigen3-dev libglm-dev \
  libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If `libglm-dev` cannot be installed system-wide, unpack the Ubuntu package
locally and pass its prefix to CMake:

```bash
mkdir -p thirdparty/glm
cd thirdparty/glm
apt-get download libglm-dev
dpkg-deb -x libglm-dev_*.deb root
cd ../..
colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/thirdparty/glm/root/usr"
```

## Local Simulation

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```

On a development machine with a stale DDS interface, use a working DDS
configuration. For example, select Fast DDS for isolated local testing:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
```

Do not use this localhost-only override on the robot. The deployed SCAN stack
keeps Go2 DDS on `eth0` and workstation DDS on `lo`, then forwards only the
visualization graph over the TCP bridge described in `必看.md`.

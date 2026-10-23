# lio_bridge

FAST-LIO2 到 SCAN-Planner 的 C++ ROS 2 适配器。

## 输入

- `/Odometry`
- `/cloud_registered_body`

## 输出

- `/LIO/odom_vehicle`
- `/LIO/odom_imu`
- `/LIO/clouds_lidar`

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/fastlio_ws/install/local_setup.bash
source install/local_setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/.config/ros2/cyclonedds.xml

ros2 launch lio_bridge fastlio_bridge.launch.py
```

FAST-LIO 的 `/Odometry` 只有约 10 Hz，且原始 twist 不适合作为控制反馈。bridge 会保留连续位姿，从位置差分重新估计线速度，并设置速度窗口、死区、跳变检测和最大速度保护。`/LIO/odom_vehicle` 与 `/LIO/odom_imu` 目前都来自 FAST-LIO 的 IMU-body 位姿。

`/Odometry` 只订阅一次；同一份位姿和速度同时发布到 `/LIO/odom_vehicle` 与 `/LIO/odom_imu`，不再为已经废弃的 LIO-SAM 双话题路径保留重复订阅和估计器。

bridge 将里程计坐标系改写为 `world` / `base`，并在 `/tf` 上随每帧发布单位变换 `base -> body`（不在 `/tf_static`）。FAST-LIO 启动时必须设置 `tf_topic:=/fastlio/tf`，否则其 `camera_init -> body` 会和 `base -> body` 冲突，使 `body` 有两个父节点并破坏 TF 树。`/cloud_registered_body` 原样以 `PointCloud2` 转发，不进入 Python 反序列化；点云仍是 `body` 系，SCAN 侧使用同一传感器位姿做 `body -> world` 变换，因此实机启动参数为 `lidar_need_extrinsic:=false`。

点云使用独立 callback group 与 `MultiThreadedExecutor`，输出为 best effort 以匹配 SCAN 的 `SensorDataQoS`；里程计输出为 reliable。Go2 上的 `CYCLONEDDS_URI` 必须只绑定 `eth0`，工作站 RViz 所需的跨机数据由 `tools/scan_wifi_bridge.py` 经 Wi-Fi TCP 转发。

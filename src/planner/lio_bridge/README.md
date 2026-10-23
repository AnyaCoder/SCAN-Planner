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

FAST-LIO 的 `/Odometry` 只有约 10 Hz，且原始 twist 不适合作为控制反馈。bridge 会保留连续位姿，从位置差分重新估计线速度，并设置速度窗口、死区、跳变检测和最大速度保护。

`/Odometry` 只订阅一次；bridge 从同一位姿生成传感器系的 `/LIO/odom_imu` 和底盘系的 `/LIO/odom_vehicle`，不再为已经废弃的 LIO-SAM 双话题路径保留重复订阅和估计器。

`/LIO/odom_imu` 是经过世界校准的 IMU-body 位姿，和 `/cloud_registered_body` 同属 `body` 系。`/LIO/odom_vehicle` 是用于底盘控制的 `base` 位姿：启动后第一帧静止位姿会建立 `IMU -> base` 安装旋转，使 `base` 从水平姿态开始；此后保留真实的 roll/pitch/yaw 变化。若启动时机器人不在水平面，将 `align_base_on_start` 设为 `false`，改用固定安装外参。

bridge 将里程计坐标系改写为 `world` / `base`，并在 `/tf` 上随每帧发布 `base -> body` 安装旋转（不在 `/tf_static`，不是单位四元数）。FAST-LIO 启动时必须设置 `tf_topic:=/fastlio/tf`，否则其 `camera_init -> body` 会和 `base -> body` 冲突，使 `body` 有两个父节点并破坏 TF 树。`/cloud_registered_body` 原样以 `PointCloud2` 转发，不进入 Python 反序列化；SCAN 侧使用 `/LIO/odom_imu` 做 `body -> world` 变换，因此实机启动参数为 `lidar_need_extrinsic:=false`。

`config/fastlio_bridge.yaml` 的 `world_alignment_*` 把 FAST-LIO 世界系左乘旋转到地面水平。当前 Go2 + Mid-360 的实测量约为 roll `-0.89°`、pitch `-1.83°`。重新标定时，先把四元数设为 `(0, 0, 0, 1)`，重启并在地面静止建图，然后在工作站运行：

```bash
python3 tools/calibrate_world_alignment.py
```

把输出的四元数写回配置并重启。校准后近机器人地面法向残差应小于 `0.2°`。不要用 `ground_height`、障碍膨胀半径或加速度容差去补地面倾斜。

点云使用独立 callback group 与 `MultiThreadedExecutor`，输出为 best effort 以匹配 SCAN 的 `SensorDataQoS`；里程计输出为 reliable。Go2 上的 `CYCLONEDDS_URI` 必须只绑定 `eth0`，工作站 RViz 所需的跨机数据由 `tools/scan_wifi_bridge.py` 经 Wi-Fi TCP 转发。

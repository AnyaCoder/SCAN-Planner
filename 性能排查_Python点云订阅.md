# Python 订阅点云性能排查

## 结论

这次看到的“LiDAR 只有 2.7 Hz、IMU 只有 3.7 Hz”是**测量端的问题**，不是 Mid-360 或 Livox 驱动只有这个频率。

- 单独用 Python 订阅 `/livox/imu`：10 秒收到 2001 条，约 **200.14 Hz**，平均时间戳年龄约 2 ms。
- 同一个 Python 进程同时订阅 `/livox/lidar`（`CustomMsg`，约 20,003 点/帧）和 `/livox/imu`：点云约 **2.74 Hz**，IMU 约 **3.74 Hz**。
- 用 `ros2 topic hz /livox/lidar` 测点云也只有约 **2.10 Hz**，同一时刻 `/livox/imu` 是 **200.43 Hz**；`ros2 topic hz` 同样是 Python 订阅端，不能用来判断 `CustomMsg` 点云的真实频率。
- Livox 驱动单独运行、没有 LIO-SAM 时，点云慢和 IMU 慢的现象同样存在，说明不是 LIO-SAM 计算拖慢。
- LIO-SAM 的 C++ `imageProjection` 日志中相邻 scan 间隔约 100 ms，说明输入点云本身基本是 10 Hz 级别。

因此，**不要用“一个 Python 节点同时订阅点云和 IMU”或 `ros2 topic hz` 的结果评估驱动或 SLAM 的实时性。**

## FAST-LIO2 接入后的补充结论

切换到 FAST-LIO2 + `/cloud_registered_body` 后，同样不能用 Python echo 判断点云链路：

- FAST-LIO 的 `/Odometry` 和 `/cloud_registered_body` 发布端均为 reliable；`ros2 topic echo` 默认 best effort，会表现为“有 publisher 但收不到”。
- Foxy 的 `ros2 topic echo` 不支持 `--once`，应使用 `PYTHONUNBUFFERED=1 timeout N ros2 topic echo ... | head`。
- C++ `lio_bridge` 实测稳定收到 `/cloud_registered_body` 约 10 Hz、每帧约 5100 点，并同步转发 `/Odometry` 约 10 Hz。
- Python probe 读取单帧原始 `/livox/lidar` 时约有 2 万点，其中按 FAST-LIO 规则过滤后约 5100 点，说明点云和过滤链路正常。

检查 FAST-LIO 里程计时必须显式指定 reliable：

```bash
ros2 topic echo --qos-reliability reliable --csv /Odometry
```

## 本次实测数据

测试条件：Go2 + Mid-360，LIO-SAM 已停止，只运行 `livox_ros_driver2`。

### 同时订阅点云和 IMU

```text
/livox/imu      n=38    rate=3.74 Hz
/livox/lidar    n=27    rate=2.74 Hz
点云点数平均    20003.6
```

### 只订阅 IMU

```text
/livox/imu      n=2001  rate=200.14 Hz
时间戳年龄均值  -2.1 ms
p95             -0.4 ms
```

### `ros2 topic hz` 的对照结果

```text
/livox/imu      average rate: 200.429 Hz
/livox/lidar    average rate: 2.101 Hz
```

IMU 同时正常，点云却只有约 2 Hz，说明瓶颈在 Python 点云反序列化，而不是网络或 LiDAR。

### LIO-SAM 正常启动后的稳定基线

```text
/livox/imu                              200.04 Hz
/odometry/imu                           166.65 Hz
/odometry/imu_incremental               200.13 Hz
/lio_sam_ros2/mapping/odometry            4.22 Hz
/lio_sam_ros2/mapping/odometry_incremental 4.22 Hz
```

`mapping/odometry` 的约 4.2 Hz 和约 350 ms 时间戳年龄是 LIO-SAM 地图优化输出本身的特征，和 Python 订阅点云无关。

## 根因

### 1. Python 反序列化 `CustomMsg` 很重

Livox 的 `CustomMsg` 每帧约 2 万点。`rclpy` 收到消息后会为点数组创建 Python 对象；每帧 2 万次对象创建，10 Hz 就是约 20 万对象/秒。对 Jetson Orin 上的 Python 进程来说，这会占用大量 CPU，并产生明显延迟。

### 2. 单线程 executor 被点云回调阻塞

当时测试脚本使用默认的 mutually exclusive callback group 和 `rclpy.spin_once()`。点云回调执行期间，IMU 回调无法运行，于是 IMU 看起来也只有 3.7 Hz。

这不是 DDS 丢包，也不代表 IMU 实际只有 3.7 Hz。

### 3. 时间戳年龄会放大误判

重回调造成消息排队时，后到的 IMU 消息时间戳会看起来“落后”数秒。只看到 `-4 s` 的 age 很容易误判成网络或驱动问题；应先排除订阅端排队。

## 短期规避

- 测 IMU 频率时，单独起一个只订阅 IMU 的进程。
- 测点云频率时，不要同时在该 Python 节点里订阅 IMU；或者至少使用多线程 executor 和独立 callback group。
- 点云统计只看 `point_num`、`header.stamp`、帧间隔，不要在 Python 里逐点遍历。
- 需要可靠数据时，用 C++ 探针或 C++ 录包节点，不要用 Python 逐点处理。
- `ros2 topic hz` 本身也大量依赖 Python 运行时；在重消息上出现异常时，以 C++ 订阅或 LIO 日志为准。

## 后续 C++ 高速实现方案

目标：在不拖慢 IMU 的情况下接收 Mid-360 点云，并给 SCAN-Planner 提供低延迟输入。

### 订阅层

- 使用 `rclcpp::Node` 和 `MultiThreadedExecutor`。
- 点云、IMU 使用不同的 `ReentrantCallbackGroup`，避免互相阻塞。
- QoS 与发布端匹配；当前 Livox 发布端是 `RELIABLE` / `VOLATILE`。
- 优先直接接收 `livox_ros_driver2::msg::CustomMsg`，在 C++ 中直接读取点数组；必要时再转 `sensor_msgs::msg::PointCloud2`。
- 对点云消息使用移动语义，避免 `std::vector` 和 `PCL` 容器的重复拷贝；输出端预留容量。

### 处理层

- 点云直接在 C++ 中完成降采样、去畸变和坐标变换，避免进入 Python。
- 固定尺寸的环形缓冲区/无锁队列连接“接收线程”和“规划线程”。
- 如果只做局部规划，先按 `grid_map.max_ray_length` 或传感器范围裁剪，不要先构造全量全局地图。
- 评估是否能把 `CustomMsg` 在驱动侧或桥接节点转成 `PointCloud2`，减少自定义消息的跨语言开销。

### 不建议的路径

- 不建议用 Python 做 2 万点/帧的逐点循环。
- 不建议在一个单线程 Python executor 中同时处理点云、IMU、里程计和规划回调。
- 不要假设 ROS 2 Foxy + CycloneDDS 一定能对自定义消息做零拷贝；需要实测消息是否发生拷贝。

## 下一步基准

用同一个 C++ 探针，在相同 FAST-LIO2/Mid-360 配置下记录：

- `/livox/lidar` 实际频率、每帧点数、帧间隔 p50/p99；
- `/livox/imu` 频率和端到端时间戳年龄；
- 点云接收线程 CPU、峰值内存、消息拷贝次数；
- FAST-LIO `/Odometry`、`/cloud_registered_body` 的输出频率和 age；
- 接入 SCAN 后的重规划耗时和控制跟踪误差。

只有 C++ 探针的数据才适合作为后续 FAST-LIO2 / Point-LIO 对比基线。

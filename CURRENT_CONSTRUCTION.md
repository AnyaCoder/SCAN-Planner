# 当前施工：相机选点与俯视图兜底

更新日期：2026-11-23

## 当前目标

保留现有的局部导航闭环：Go2 前置相机是普通场景的路径落点来源，双 LiDAR
生成 10 m x 10 m、5 cm 分辨率的局部俯视图。相机贴近墙面或看不到地面时，
VLM 改从俯视图灰色可走区域选择落点；所有选中点最终由 SCAN-Planner 负责
碰撞检查和运动执行。

本次已回退 Mission/B2D3D 相关改动，不再启动 Mission Manager，也不维护 Visit
队列、子任务拆解、完成确认或跨 Visit 上下文。相机与俯视图使用当前帧，历史只
保留已审批的世界点，避免图像上下文膨胀。

## 固定边界

1. SCAN-Planner 是碰撞检查、路径规划和运动执行的唯一安全层。
2. VLM 优先从相机选择地面点，不判断机器人是否能通过。
3. 相机贴近墙面、玻璃或找不到地面时，VLM 返回 `topdown`，再从俯视图的
   中性灰色地面区域选择一个归一化点。
4. 所有目标点都复用原有鱼眼去畸变、相机 TF、地面拟合和 LiDAR 射线落点代码。
5. 俯视图固定为机器人 IMU 附近的 10 m x 10 m 局部窗口，不使用全局地图替代。
6. 点云持续在 Go2 内存中滚动；只有 VLM 请求或重选时才生成一张新截图。
7. 历史不保存旧相机图和旧俯视图，只保存已审批落点的时间与世界坐标。

## 当前流程

1. `start_scan_stack.sh` 启动 LIO、双 LiDAR、SCAN、前置相机和 RViz。
2. `start_vlm_pixel_shadow.sh` 启动滚动点云视图与 VLM 落点节点。
3. 第一段只发送带归一化坐标网格的前置相机图。相机可见地面时返回
   `point: x y`；贴墙、玻璃或没有可见地面时返回 `topdown`。
4. 收到相机点时，程序复用原有反投影：
   `cv2.fisheye.undistortPoints`、相机到世界 TF、`ground_plane()` 地面求交、
   `lidar_target_for_ray()` 邻域 LiDAR 落点，以及 `distance-frame` 下的距离
   限制。
5. 相机选点反投影失败时也自动进入俯视图兜底。该请求同时发送相机和
   10 m x 10 m 俯视图，VLM 从灰色可走区域返回一个归一化点。
6. 程序检查该点周围 0.20 m 内存在中性灰色地面，再按地图原点和机器人位姿
   转换为世界点；SCAN-Planner 仍负责碰撞检查和执行。
7. 结果先在 RViz 中以十字和目标点显示；操作员批准后才交给 SCAN-Planner。
   相机模式即使落点在左右方向，也不会因它直接触发原地转弯。
8. 已批准世界点写入简短历史和轨迹叠加；“清空记忆”同时清除文字历史和图上点。
9. “让 VLM 重选”会立即取消旧生成并重新截取相机与俯视图，旧结果不能覆盖新结果。

## 当前已有

- 双 LiDAR 时间同步、外参转换和 IMU 中心坐标。
- 地面灰色、障碍绿色/青色、无回波黑色显示。
- 相机世界点会投影回俯视图，仅用于可视化、审批和轨迹记忆。
- 天花板过滤、自遮挡双圆柱、前进箭头和 10 m 范围裁剪。
- VLM 流式坐标快速显示和 RViz 审批面板。
- 已审批世界点历史与对应轨迹可视化。
- USB Wi-Fi、热点模式和远程 VLM 启动链路。
- 相机选点、贴墙后俯视图兜底的独立提示词和实际请求图像记录。
- 调试脚本集中在 `debug/tools/`，按需截图和标定输出分别写入
  `debug/pointcloud_views/`、`debug/calibration/`，正式的启动和控制入口仍
  保留在 `tools/`。

## 已知限制

当前使用的通用视觉语言模型没有针对本项目的俯视图染色、尺度和双圆柱表示
进行训练。实机近墙场景中，它经常把墙边或障碍边缘当作灰色可走区域，俯视图
落点不能作为稳定依据。当前主路径仍是前置相机选点和 LiDAR 反投影；俯视图
继续用于可视化、审批和人工判断，后续若用于自动落点应先做针对性训练或采用
几何可行区域约束，不能靠继续堆提示词解决。

## 接口

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/vlm_pixel_shadow/status` | `std_msgs/String` | 落点、审批和运动状态 |
| `/vlm_pixel_shadow` | `visualization_msgs/MarkerArray` | 世界坐标目标、轨迹和状态标记 |
| `/go2/front_camera/vlm_point_overlay` | `sensor_msgs/Image` | 相机落点叠加图 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 审批后发送给 SCAN 的地图目标 |

## 回归要求

```bash
source /opt/ros/foxy/setup.bash
/usr/bin/python3 -m py_compile \
  src/planner/plan_manage/scripts/vlm_common.py \
  src/planner/plan_manage/scripts/vlm_pixel_shadow.py
colcon build --packages-select scan_planner --symlink-install
/usr/bin/python3 -m pytest -q src/planner/plan_manage/test
```

真机启动与停止使用：

```bash
./tools/start_scan_stack.sh
./tools/start_vlm_pixel_shadow.sh
./tools/stop_scan_stack.sh
```

## 后续工作

- 在真实办公室场景中继续验证地图落点与相机语义的一致性。
- 转弯、后退和重选后的运动继续全部交给 SCAN-Planner，不由 VLM 接管底层
  运动安全。
- 长程任务结构尚未接入；如后续重新设计，应作为独立任务层评审，不能混入当前
  局部落点选择流程。

# 当前施工：相机选点、贴墙判定与原地转向

更新日期：2026-11-23

## 当前目标

保留现有的局部导航闭环：Go2 前置相机是普通场景的路径落点来源，双 LiDAR
生成 10 m x 10 m、5 cm 分辨率的局部俯视图。相机贴近墙面或看不到地面时，
VLM 结合前置相机和俯视图估计一个相对狗当前朝向的原地转角。操作员批准后，
Go2 通过原生 Unitree Sport 接口原地转向；转完立即清除旧目标，重新抓取相机
和俯视图并回到正常选路。普通前进落点仍由 SCAN-Planner 负责碰撞检查和运动
执行。

本次已回退 Mission/B2D3D 相关改动，不再启动 Mission Manager，也不维护 Visit
队列、子任务拆解、完成确认或跨 Visit 上下文。相机与俯视图使用当前帧，历史只
保留已审批的世界点和转角，避免图像上下文膨胀。

## 固定边界

1. SCAN-Planner 是碰撞检查、路径规划和运动执行的唯一安全层。
2. VLM 优先从相机选择地面点，不判断机器人是否能通过。
3. 相机贴近墙面、玻璃或找不到地面时，VLM 返回 `topdown`。第二次请求看
   相机和俯视图，只估计原地转向角，不再从俯视图选落点。
4. 所有目标点都复用原有鱼眼去畸变、相机 TF、地面拟合和 LiDAR 射线落点代码。
5. 俯视图固定为机器人 IMU 附近的 10 m x 10 m 局部窗口，不使用全局地图替代。
6. 点云持续在 Go2 内存中滚动；只有 VLM 请求或重选时才生成一张新截图。
7. 原地转向复用已验证的 Unitree Sport 直通接口，默认 `1.0 rad/s`、
   100 Hz；不发布 SCAN goal，也不使用 `/cmd_vel` 代替原生转向。
8. 历史不保存旧相机图和旧俯视图，只保存已审批落点的时间、世界坐标，以及
   已审批转向的时间与转角。

## 当前流程

1. `start_scan_stack.sh` 启动 LIO、双 LiDAR、SCAN、前置相机和 RViz。
2. `start_vlm_pixel_shadow.sh` 启动滚动点云视图与 VLM 落点节点。
3. 第一段只发送带归一化坐标网格的前置相机图。相机可见地面时返回
   `point: x y`；贴墙、玻璃或没有可见地面时返回 `topdown`。
4. 收到相机点时，程序复用原有反投影：
   `cv2.fisheye.undistortPoints`、相机到世界 TF、`ground_plane()` 地面求交、
   `lidar_target_for_ray()` 邻域 LiDAR 落点，以及 `distance-frame` 下的距离
   限制。
5. 相机选点反投影失败时也自动进入贴墙分支。该请求把相机和 10 m x 10 m
   俯视图按顺序发送给 VLM，要求它找出最近的连续灰色通道，并只返回：

   ```text
   turn: angle_deg
   ```

   正角为相对狗头方向的左转，负角为右转，`±180` 为调头。
6. 程序把角度规范到 `[-180, 180]`，拒绝小于 5° 的无意义动作。结果显示为
   相机图提示、俯视图橙色转角和 RViz 圆弧，操作员批准前狗保持停止。
7. 批准后异步调用 `tools/run_sport_turn.sh`。执行期间不发起新的 VLM
   请求，重复批准和重选会被拒绝；执行完成或失败都会解除锁定，旧目标清空，
   随即重新抓取相机图。
8. 转向后的相机图仍按正常流程选择地面前进点，由 SCAN-Planner 规划并执行。
9. 已批准前进世界点继续写入简短历史和轨迹叠加；已批准转角也写入文字历史，
   但不写入世界轨迹点。“清空记忆”同时清除两者。
10. “让 VLM 重选”会立即取消旧生成并重新截取相机与俯视图，旧结果不能覆盖
    新结果。若用户在流式早期坐标出现后批准，后台剩余生成也会失效，不能再次
    覆盖已批准动作。

## 当前已有

- 双 LiDAR 时间同步、外参转换和 IMU 中心坐标。
- 地面灰色、障碍绿色/青色、无回波黑色显示。
- 相机世界点会投影回俯视图，仅用于可视化、审批和轨迹记忆。
- 天花板过滤、自遮挡双圆柱、前进箭头和 10 m 范围裁剪。
- VLM 流式坐标快速显示和 RViz 审批面板。
- RViz 审批面板区分普通 SCAN 落点和原地转向，并显示转向执行结果。
- 已审批世界点历史与对应轨迹、转角历史可视化。
- USB Wi-Fi、热点模式和远程 VLM 启动链路。
- 相机选点、贴墙后俯视转角的独立提示词和实际请求图像记录。
- Unitree Sport 原生原地转向，支持 100–300 Hz 指令和完整 360° 连续角度。
- 调试脚本集中在 `debug/tools/`，按需截图和标定输出分别写入
  `debug/pointcloud_views/`、`debug/calibration/`，正式的启动和控制入口仍
  保留在 `tools/`。

## 已知限制

通用视觉语言模型没有针对本项目的俯视图染色、尺度和双圆柱表示进行训练。
实机近墙场景中，转角可能偏大或偏小，因此仍保留人工审批，不自动闭环。当前
版本只解决平地贴墙后的朝向修正，不平滑处理斜坡、台阶或崎岖地形；这些场景
应继续由几何层和后续局部 2.5D 高程图处理。

## 接口

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/vlm_pixel_shadow/status` | `std_msgs/String` | 落点、审批和运动状态 |
| `/vlm_pixel_shadow` | `visualization_msgs/MarkerArray` | 世界坐标目标、轨迹和状态标记 |
| `/go2/front_camera/vlm_point_overlay` | `sensor_msgs/Image` | 相机落点叠加图 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 审批后发送给 SCAN 的地图目标 |
| `/go2/vlm_approval` | `std_msgs/String` | `approve`、`reselect`、`clear_memory` |

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

- 在真实办公室贴墙场景中验证 VLM 转角方向、俯视图朝向和 Sport 实转结果；
  不做主动碰撞测试。
- 原地转向后验证旧目标清理、相机重抓和 SCAN 新目标的时序。
- 普通前进、后退和 SCAN 路径仍全部交给 SCAN-Planner，不由 VLM 接管路径
  规划和碰撞安全。
- 长程任务结构尚未接入；如后续重新设计，应作为独立任务层评审，不能混入当前
  局部落点选择流程。

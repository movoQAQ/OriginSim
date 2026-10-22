# TDT 代价图显示与锐角跟踪修正（2026-10-23）

## 可视化依据

参考 [T-DT-Algorithm-2026/tdt-nav-kit 的 YAstar::getCostMapImage](https://github.com/T-DT-Algorithm-2026/tdt-nav-kit/blob/814d5ce0237acc7faa8c070e5c5e61e2ddc81996/src/YAstar/yastar.cpp)。上游直接将真实浮点 costMap 限幅为图像，并没有一套彩色 ESDF 公式；本项目采用这个“实际代价转图像”的思路，再用标准 OpenCV JET 查表着色，不将自己生成的彩色表冒称为 TDT 原版。

当前规划代价约为 1..1.56，硬碰撞为无穷大，因此显示采用固定范围 1..2：有限代价编码 round(99×clamp((cost−1)/(visualization_max_cost−1),0,1))，无穷为 100。visualization_max_cost 默认 2，可在 bringup/params/params.yaml 修改。色表使用 OpenCV JET 的 80..224 区间，去掉深蓝尾部，表现为浅蓝、绿、黄、红。该查表是本项目彩色显示扩展；地图本身仍由 MapDisplay 纹理栅格绘制。

不再按碰撞边界的理论值自动拉伸、不随每帧最大值重标度、不将距离值代替实际规划代价。运行节点逐格核对 174,900 个栅格通过，详见 rmuc2026_live_grid.json。显示只读，不影响规划浮点数据；原始 2026 YAML/PGM、0.30 m 半径、Depth 为空均保持。

## 锐角根因与修正

原参考速度只约束 v²κ（横向加速度），固定车体朝向时没有约束 vκ（行进方向变化率）。底盘使用真实舵轮：转向速度上限 3 rad/s、转向加速度 10 rad/s²、转向增益 6，轮速变化也受限。车体不转并不代表轮组能瞬间改变运动方向。

PathTrajectory 现在同时约束 v≤sqrt(a/κ) 和 v≤reference_yaw_rate/κ，随后执行原有前后向加减速传播，因此在进入急弯前降速。reference_yaw_rate=1 rad/s 同时约束车体参考朝向和行进方向变化率，在小陀螺模式下仍生效。保留 q_cross=12、reference_speed=1.2 m/s、reference_acceleration=1 m/s²、control_delay_compensation=0.05 s。没有通过放松碰撞检查实现跟踪。

密集栅格锐角回归包含实际 0.02 m/s 输出死区，确认不会因为新增降速卡在弯点。53 项地图和控制回归通过（地图 47、控制器 6）。

## 真实 Gazebo 接触模型对照

使用 hxy 原有 SwerveDrive、轮胎和整车 SDF，在独立 ROS 域 94 与独立 Gazebo partition 的平地场景测试。MPC/reference 直接驱动真实舵轮接触物理；测量来自 Gazebo odometry，不是积分指令推算。修改前使用 095db9e 中的 PathTrajectory 与相同 q_cross=12、dt=0.02、horizon=25；修改后加入方向变化率约束。

| 路径 | 修改前最大横向偏差 | 修改后最大横向偏差 | 修改后终点误差 |
| --- | --- | --- | --- |
| 135° 折线急转 | 0.0575 m | 0.0382 m | 0.0096 m |
| 半径 0.12 m、150° 平滑急弯 | 0.0237 m | 0.00245 m | 0.0093 m |

两种路径均到达终点，求解全程成功。原始记录为 gazebo_acute_before/after.json 和 gazebo_smooth_before/after.json；绘图为 gazebo_turn_comparison.png。接触物理与回调采样会产生运行间波动，这些是单次对照数据，并非统计上界。

复现修改后场景（hxy 根目录，先 source ROS/Gazebo/nav）：

```bash
python3 nav/tools/verify_gazebo_turn.py --probe-binary nav/build/minco_controller/gazebo_tracking_probe --output-dir .task-work/recheck-acute --mode acute
python3 nav/tools/verify_gazebo_turn.py --probe-binary nav/build/minco_controller/gazebo_tracking_probe --output-dir .task-work/recheck-smooth --mode smooth
```

测试覆盖参考参数化、MPC 求解和真实底盘接触，不覆盖全局规划、Point-LIO 定位、动态障碍或 RMUC 地形。原有 2026 STL 与用户 PGM 的局部几何差异仍然存在；出现 A* start is hard-blocked 时不应误判为控制器已经可以穿越。当前运行节点参数见 active_acute_controller.json。

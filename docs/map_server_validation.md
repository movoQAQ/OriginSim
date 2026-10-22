# 验证记录（2026-10-22）

工程、构建和运行均位于 `/home/lyu/hxy`。旧 nav 的源码与构建目录已归档到 `.repository-history/`；本次测试日志位于 `.task-work/`。

## 框架与回归

- ROS 2 Jazzy / Ubuntu 24.04；22 个功能包完整构建通过，最终代码清理后复核通过。
- 地图 Core 30 项、ROS 适配器 5 项、MapClient 6 项回归通过；覆盖不可变旧视图、静态缓冲共享、生产者重启、跨版本拒绝、非法消息恢复、最新动态障碍撤回、双时钟心跳过期、Worker 错误准入，以及几何不变时通过轻量状态推进新观测。
- 使用实际 `rmuc_2027.yaml/pgm/pcd` 的 ROS 回放 19 项检查通过，包括独立云/雷达有效期、自身过滤、测量时刻 TF 缺失与恢复、静态消息只发送一次、无重复静态/健康载荷、Worker 正常退出。
- PGM 中的未知区域在二值规划图中保守作为障碍，不修改原始 PGM。

## 真实 Gazebo

运行 hxy 的 rmuc_2027 场景、NVIDIA RTX 5060 Laptop GPU、真实仿真激光和 IMU，经过 Point-LIO、map→odom、MapServer、栅格规划、Minimum Jerk 平滑、MPC 和底盘速度转换。没有用合成点云代替此项验证。

第一段目标在 x 方向前进 1.2 m，定位位移约 0.94 m，目标最小误差 0.26 m。第二段同时核对 Gazebo 真值：

| 指标 | 观测 |
| --- | --- |
| 起点（map） | (4.416, 9.505) m |
| 目标（map） | (5.616, 9.505) m |
| 停车后的定位位置 | (5.481, 9.530) m |
| 停车后目标误差 | 0.137 m |
| Gazebo 真值位移 | 0.984 m |
| 非零底盘速度命令 | 90 条 |
| 原始 / 平滑路径 | 14 / 97 点 |
| 云来源 / Worker | VALID / 无错误 |
| 静态消息 / Depth | 接收 1 次 / 空数组 |

最终修复版本通过一键冷启动复核及 8 项真实 Gazebo 条件检查：真值位移 1.120 m，停车后目标误差 0.064 m，原始/平滑路径 28/171 点。

复核命令（先启动仿真与导航，继承相同 ROS_DOMAIN_ID）：

```bash
cd /home/lyu/hxy/nav
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/verify_gazebo.py --report /home/lyu/hxy/.task-work/gazebo-validation.json
```

## 边界

RadarTargets 为测试回放输入，未验证真实外部雷达位段协议。Depth、Height、Region 图均未启用；现有 topo 区域坐标未核对 rmuc_2027，因此 `use_topology: false`，本次真实闭环验证的是栅格搜索、平滑与控制链，未验证跨区域/台阶语义。实车与长期高负载验证不在此次 Gazebo 结果的覆盖范围内。

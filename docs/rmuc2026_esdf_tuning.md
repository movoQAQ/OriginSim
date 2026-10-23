# rmuc2026 原图、机器人半径与 ESDF 调整

实际默认启动采用用户指定的 `src/bringup/map/rmuc_2026.yaml` 和原始 PGM（583×300，0.05 m/像素）。PGM/YAML 未重采样、未改墙体、未被 2027 地图替换。hxy 的 `gazebo/src/rmu_gazebo_simulator/config/gz_world.yaml` 已切换 `world: rmuc_2026`。

| 参数 | 原值 | 当前值 |
| --- | --- | --- |
| robot_radius | 0.43 m | **0.30 m（用户指定）** |
| safety_margin | 0.05 m | 0.05 m |
| soft_radius | 1.5 m | 0.8 m |
| alpha | 1.0 | 0.7 |
| beta | 0.1 m | 0.15 m |

半径同步到 MapServer/Core/ROS 默认值与 MapClient 配置/默认值。规划、平滑、控制器预测碰撞和路径阻塞检查共同读取 MapClient 的 0.30 m 足迹。

## 2026 原图与通道

上侧像素列 377..393、下侧列 188..204，白色宽度均约 0.85 m，最大中心距障碍像素中心 0.45 m。旧硬阈值 0.51536 m 封闭通道；当前阈值为 0.30+0.05+0.05/√2=0.38536 m。真实 PGM 上限制在两条窄槽内的 YAstar 搜索均成功，保证不是从其他位置绕行。结果见 rmuc2026_tunnels.json 和 test_rmuc2026.cpp。

按当前 0.30 m 半径统计：可通行栅格 84,351 个，距离中位数 0.75 m。保持该半径不变，旧软代价参数只有 4,666 个栅格为基础代价 1，新参数增至 39,847 个（47.2%），最大有限代价约 1.5582；单独调整软代价不会改变碰撞掩码。距离 0.60 m 时，代价从约 1.804 降至 1.196，达到 0.80 m 后代价为 1。结果见 rmuc2026_costs.json。

## bit-nav 距离色带与栅格显示

当前显示采用 bit-nav 的原生 RViz Rainbow、距离自动范围和 0.6 透明度，仍按纹理栅格绘制。详见 [bit_nav_esdf_display.md](bit_nav_esdf_display.md)。此前固定代价 1..2 的 TDT 显示方案已替换，规划参数保持原值。

## 2026 PCD 与验证边界

本机没有配套 2026 PCD，因此从现有 2026 Gazebo 静态碰撞表面生成参考 PCD，包含主场地、减速带和外围墙。使用 SDF 中的 mesh scale、模型/连杆/碰撞姿态，世界坐标输出；0.04 m 体素采样、固定随机种子 2026，1,364,092 点。来源哈希、坐标范围与输出哈希保存在 bringup/map/rmuc_2026.metadata.json。复现命令（在 hxy 根目录）：

```bash
python3 nav/tools/sample_gazebo_reference.py --world gazebo/src/rmu_gazebo_simulator/resource/worlds/rmuc_2026_world.sdf --models gazebo/src/rmu_gazebo_simulator/resource/models --output nav/src/bringup/map/rmuc_2026.pcd
```

现有 2026 STL 场景和用户原始 PGM 的局部障碍形状、边界范围并非完全一致。未改动用户 PGM 去迎合 STL，也未缩放 Gazebo 车模；0.30 m 是用户指定的导航半径，不能解释为对现有仿真车轮外形的重新标定。两侧原图穿槽规划成功不等于相同位置的物理通道已经通过 Gazebo 验证。

53 项地图/控制回归通过，包括新增原图参数、两侧穿槽、真实坐标搜索三项。运行中参数检查确认 2026 YAML/PCD、服务端/客户端 0.30 m 半径以及 Depth 为空。当前距离热力图核对见 bit_nav_live_grid.json。Gazebo 运动与跟踪结果另存报告；不能用历史 2027 结果替代当前 2026 场景结果。

当前 2026/0.30 m 运动验证：一次约 6 m 目标真值位移 6.077 m，终点误差 0.087 m，该次有效规划后路径清空次数为 0。它包含绕行段，最大横向偏差 0.401 m，因此不能用该报告声称跟踪精度已收敛。另一转弯诊断失败，期间发生新目标切换、里程计超过 0.5 s 超时以及后续起点硬碰撞，最终目标未到达；失败报告也保留。新一轮手工目标中仍出现 EXECUTION_STALLED，需要继续处理地图/场景几何差异与定位/跟踪时序，不能将配色、半径和原图通道搜索通过等同于整套导航已稳定。

上述整图运动失败属于历史诊断。最新参考速度修正已在独立真实底盘平地场景验证，详情见 tdt_visualization_acute_tracking.md；未将局部跟踪对照等同于解决地图/场景几何差异。

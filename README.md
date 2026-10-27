# OriginSim nav — hxy 集成

[接口、资源与运行说明](docs/map_server_usage.md) · [迁移记录](docs/map_server_migration.md) · [验证记录](docs/map_server_validation.md)

地图统一使用 MapClient 和只读 MapView；topology_global_planner 是普通 ROS 2 功能包。按用户要求固定使用原 bringup/map/rmuc_2026.pgm 和 rmuc_2026.yaml；不重制 PGM，仅允许调整 YAML 原点以匹配 PCD；机器人半径 0.30 m，Depth 未启用。ESDF 用浅蓝/绿/黄/红的纹理栅格显示。当前地图尚无已验证匹配的区域/拓扑配置，拓扑约束暂时关闭。

当前无 smoother 穿洞配置、测试结果与运行边界见 [五洞车辆与算法验证](docs/no_smoother_vehicle_tunnels.md)。
仓库自带的五洞 × 三偏移回归、独立雷达/IMU/轮速断流测试及完整构建命令见
[自动化回归工具](docs/tunnel_regression.md)。历史 15/15 与新工具的每次运行结果分别保存。

老图与 PCD 的原点配准、验证及剩余几何差异见 [老图配准说明](docs/original_map_alignment.md)。

新增的配套入口为 `ros2 launch rmu_gazebo_simulator tunnel_sim.launch.py` 和
`ros2 launch bringup no_smoother_tunnels.launch.py initial_x:=... initial_y:=... initial_z:=...`。
后者使用轮速/IMU/原始雷达围墙融合定位与行进航向控制；起步标定和范围见报告。
不要与原 Point-LIO startup 或另一个导航执行器同时启动。

T-DT Z-Localization 后端的独立接入入口、构建步骤和仿真验收见
[T-DT 定位后端接入](docs/tdt_localization_sim.md)。
使用 Z-Localization 仓库自带 ZLIO 前端的仿真入口与验收见
[T-DT + ZLIO 仿真](docs/tdt_zlio_sim.md)。
Adaptive-LIO 前端与 T-DT 后端的 Gazebo 接入及真值验收见
[T-DT + Adaptive-LIO 仿真](docs/tdt_adaptive_lio_sim.md)。
T-DT 三种前端的仿真定位误差统计与轨迹诊断见 [定位精度报告](docs/tdt_localization_accuracy.md)。

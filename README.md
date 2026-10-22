# OriginSim nav — hxy 集成

[接口、资源与运行说明](docs/map_server_usage.md) · [迁移记录](docs/map_server_migration.md) · [验证记录](docs/map_server_validation.md)

地图统一使用 MapClient 和只读 MapView；topology_global_planner 是普通 ROS 2 功能包。默认使用用户指定的 bringup/map/rmuc_2026.yaml 和配套 2026 PCD；机器人半径 0.30 m，Depth 未启用。ESDF 用浅蓝/绿/黄/红的纹理栅格显示。当前地图尚无已验证匹配的区域/拓扑配置，拓扑约束暂时关闭。

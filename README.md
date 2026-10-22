# OriginSim nav — MapServer refactor

基线与迁移：[迁移记录](docs/map_server_migration.md)。

构建、资源、接口与时序：[使用说明](docs/map_server_usage.md)。

实际验证结果、性能与限制：[验收报告](docs/map_server_validation.md)。

topology_global_planner 已作为普通 ROS 2 功能包直接纳入主仓库，不再使用 Git 子模块；普通 clone 即可获得完整导航源码。实车运行前仍需提供匹配的 PCD、2026 拓扑、区域图及真实雷达上游，具体缺口见验收报告。

# MapServer 重构迁移记录

基线：OriginSim 远程 nav c905df17ff5827629db680833a7e931e616dc6aa；拓扑子模块 55a4f87c1fc1605ed24c23d402b2277a819eecb4。
拓扑规划器现已转为主仓库直接管理的普通 ROS 2 功能包，不再保留 gitlink 或 .gitmodules；上述提交号用于追溯来源。

当前用户要求覆盖附件中的逐阶段测试：完整框架形成后才构建及执行测试。

| existing | target |
|---|---|
| static_map_node / ROS static_map_loader | Core static_map / strict aligned layer loader |
| dynamic_map_node / odom accumulation | ROS cloud_adapter at measurement time + Core map-frame worker |
| DynamicCloudFilter / DynamicMemory | 原算法保留；内部 binary 0/1；固定 map PCD / 地图坐标差分 |
| global/local costmap_server_node / layers | 单个 map_server_node + map_server_core Worker |
| uint8 Costmap.msg | DerivedMap.msg: 完整同版本 float/binary 数据 |
| CostmapClient / mutable Costmap2D | ROS consumer adapter / 共享不可变 DerivedMaps 视图 |
| NavFn byte cost path | topology_global_planner::grid::YAstarCore float-input A* |
| smoother second inflation | 同版本 CollisionMask 反转为 SFC 0=障碍 / 255=自由 |
| executor global/local clients | 单一 map_client，规划线程持有固定共享版本 |
| route monitor waits for derived map | 轻量 DynamicObstacles 输入可立即叠加检查，无全场 float 拷贝 |

## 边界与真实缺口

- MapServer 没有拓扑图、Connector 搜索、SFC 或控制逻辑。Region/Height/Depth 只查询。
- 雷达实际消息未在上游串口/裁判消息中发现完整的己敌目标列表；RadarTargets 是新增的明确接入协议，不代表实车已接通。两个 Topic 独立，map 坐标，ROS 观测时间；Core 接收时间单独采用 steady clock。按来源完整列表替换；stable_ids=false 明确降级，禁止宣称身份过滤已验证。
- PCD、Region uint16 图与 Height/Depth 不在该远程分支；static_pcd 必须显式提供。没有 PCD 时 Cloud 为 MISSING/ERROR；默认 map_client.require_cloud=true，执行器不能宣称感知健康。区域图缺失返回 INVALID_REGION；旧多边形拓扑保留，配置 use_region_raster=true 才绑定只读 regionAt，不制造区域文件。
- 严格 PGM 加载拒绝未知灰度；不能为让启动通过而擅自把未知当自由。地图资源需人工确定并验证 PCD/PGM 叠图。
- 远程分支没有 YAstar。新 YAstarCore 是本次独立实现的 C++17 浮点地图 A* 接入核心，非直接复制 TDT 上游 YAstar；保留旧规划器外部入口和拓扑分段，边权为米制边长×端点代价均值，inf 禁入，对角扩展同时检查两正交格。没有声称 TDT 上游实现已移植。
- 规划和 SFC 固定读取同一 DerivedMaps，原 NavFn 字节势场、双 Costmap 节点及旧 Layer 生产链移除。控制器的物理轮廓检查使用未膨胀 fused_obstacles；优化软代价从连续 cost 计算 1-1/c，不再除以 254。
- ROS 时钟回退统一清空动态来源；定位 map→odom 跳变只清空 Cloud 记忆/累积。TF 发布权和定位算法未修改。
- input_version 是已接收/应用的来源状态版本；version 是一次完整融合+EDT+Costmap 的障碍版本，障碍不变时不伪造新地图版本。health_at_build 是构建时健康；/map_server/health 与 /map_server/dynamic 独立更新当前健康，消费者配置是否允许缺失来源。

## 执行顺序

先完成 Core、ROS、消费方、Launch/CMake/config；随后添加/运行契约测试、包构建、集成测试与性能基准。测试结果另见最终验收记录，不凭参数频率推断性能。

全仓构建补充：Point-LIO 原安装规则要求安装仓库未分发的两个 PCD；改为存在时安装、缺失时 CMake 告警。没有创建占位地图；实际定位运行仍需要真实资源，状态估计与 TF 发布权未修改。bringup 移除不存在的 mppi_controller 依赖，保留实际 minco_controller。

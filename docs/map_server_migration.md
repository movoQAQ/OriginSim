# 本次迁移

OriginSim nav 分支的框架已接入 `/home/lyu/hxy/nav`；原 hxy nav 源码和构建目录保存在 `/home/lyu/hxy/.repository-history/`。在 hxy 内构建和运行。

- 删除旧的可写地图类、客户端双层地图状态与兼容接口，统一为 MapClient 发布只读 MapView。
- 规划器、平滑器、执行器和 MPC 直接消费不可变视图。
- 静态栅格、派生地图、动态几何和健康心跳拆开；不在每次派生或动态消息中携带静态语义或重复健康数据。
- ROS 生产节点共享 Core 的 StaticMap，取消第二次静态地图加载。
- topo 作为普通功能包；无 Git 子模块。
- 暂时保留 RadarTargets 输入。仿真使用 hxy rmuc_2027 PCD/PGM，Depth 留空。

使用和边界见 [接口说明](map_server_usage.md)，实际验收证据见 [验证记录](map_server_validation.md)。

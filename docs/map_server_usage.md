# MapServer 使用与数据契约

本实现基于 OriginSim `nav` 的 `c905df17ff5827629db680833a7e931e616dc6aa`，原拓扑规划器来源为 `55a4f87c1fc1605ed24c23d402b2277a819eecb4`。
拓扑规划器已从 Git 子模块转换为主仓库直接管理的普通 ROS 2 功能包。其源码、配置及本次地图接口修改随主仓库一起提交，clone 后无需 submodule init/update。上述子模块提交号仅记录代码来源。

## 构建

在项目根目录、ROS 2 Jazzy / Ubuntu 24.04 环境：

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ROS_LOG_DIR="$PWD/log/ros" colcon test --packages-select map_server --event-handlers console_direct+
colcon test-result --verbose
```

原工程平滑器会在缺少系统 OsqpEigen 时下载固定版本 OSQP/OsqpEigen 及 QDLDL。离线构建需事先准备对应源码缓存；测试使用的缓存路径和命令在交付验收报告中列明。不要擅自替换其固定版本。

纯 C++ Core 可独立编译，无 `rclcpp` / TF / Nav2 依赖：

```bash
cmake -S src/map_server -B build-core -DMAP_SERVER_CORE_ONLY=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-core -j2
ctest --test-dir build-core --output-on-failure
```

## 启动与资源

`ros2 launch map_server map_server.launch.py` 接受 `map`（必填 PGM/YAML）、`static_pcd`、`region_yaml`、`params_file`、`use_sim_time`。`static_pcd` / `region_yaml` 参数非空时覆盖配置文件，否则沿用文件值。`bringup` 的 `startup.launch.py` 同样暴露这两个资源参数。

- `static_pcd` 是独立配置的、已处于 `map` 的文件路径，不再隐式查找定位包 PCD。
- 仓库中的 `rmuc_2026.pgm` 几何为 583×300，0.05 m/格，原点 [-3.475, -8.975]。PGM 与实际 PCD 的坐标一致性仍需叠图确认。
- 没有 PCD 时仍提供静态地图和有效雷达派生地图，但 Cloud 不被标为健康；默认客户端 `require_cloud: true` 会阻止使用缺失点云的地图执行运动。
- `radar.self_id: -1` 表示未提供本车身份，自障碍过滤未验证。实车需设置真实 ID 并核对上游 ID 域。
- Region 文件为独立 YAML：`image` 指向 16 位单通道 PNG，`resolution` / `origin` 必须与占据图相同，`labels` 是图中所有非零标签的完整、不重复列表。0 为无效区域；语义层允许覆盖障碍。
- Height/Depth 是对应几何的单通道 float32 TIFF 及 YAML；必须为有限值，只做查询。
- 默认 Bringup 引用的 `topo26.yaml` 未随远程分支分发；现有 `topo25.yaml` 仅用于初始化冒烟测试，不能冒充经过验证的 2026 拓扑。2026 运行需提供匹配拓扑文件。
- 缺失 Region 图返回 `INVALID_REGION`；拓扑规划器默认继续使用原有多边形配置。提供并验证标签映射后，将 `nav_executor.use_region_raster` 设为 true，绑定 MapServer `regionAt()`。目前仅按已有拓扑 `R<数字>` ID 映射，不自动创建图或 Connector。

## Topic 与时序

| Topic | 类型 | 契约 |
|---|---|---|
| `/cloud_registered`（可配） | `sensor_msgs/PointCloud2` | 非零采样 stamp，float32 XYZ，按该时刻 TF 变换至 map；Core 在 map 内差分/积累/记忆 |
| `/radar/allies`（可配） | `map_server/RadarTargets` | 独立来源完整列表，map XY，稳定 ID；本车按 ID 排除 |
| `/radar/enemies`（可配） | `map_server/RadarTargets` | 独立来源完整列表，map XY；与己方最终同样作为圆形硬障碍 |
| `/map_server/derived` | `map_server/DerivedMap` | 同版本全场二值障碍、float 米制距离、float 代价、CollisionMask；生产实例 ID + 障碍版本 |
| `/map_server/dynamic` | `map_server/DynamicObstacles` | 轻量当前 Cloud mask、己敌圆形列表及健康状态，独立于 EDT 发布 |
| `/map_server/health` | `map_server/DynamicHealth` | 三来源状态、观测年龄、身份/时间降级标志及 Worker 错误标志 |
| `/map_server/costmap` | `nav_msgs/OccupancyGrid` | 仅调试可视化，不是规划输入 |
| `/map` | `nav_msgs/OccupancyGrid` | 确定的静态二值占据可视化 |

实际雷达硬件上游没有接通：`RadarTargets` 为显式新增协议。上游必须提供 ROS 时间域中的观测 stamp，各目标 `observed_at` 不晚于帧 stamp；不能将设备/裁判绝对时间直接当作 ROS 时间。缺少采样时间时，上游应显式使用 ROS 接收时刻并设置 `time_degraded: true`。无稳定 ID 时设置 `stable_ids: false`，采用每来源完整列表替换，身份过滤未验证。

己敌空列表撤销本来源；断流保留缓存并按本来源观测时间衰减。默认 stale=2.5 s、半衰期=2 s、最大年龄=4 s、最低置信度=.25；稳态接收时间也独立检查断流。点云默认 timeout=.5 s、记忆最长1 s。消息迟到不回滚已接受的来源状态。ROS 时间回退清空所有动态来源；map→odom 定位跳变只清空 Cloud 记忆/积累，不清除雷达。

`version` 是一次完整融合/EDT/Costmap 的障碍版本；`input_version` 是已应用来源状态版本。健康变化不冒充地图变化，`health_at_build` 仅代表构建时健康。独立进程的 `producer_id` 防止重启后的版本重新计数被误判为旧图，并禁止使用跨实例组合的派生地图和健康心跳。

## C++ 与规划消费

`MapServer` 的文档目标接口均在 `include/map_server/map_server.hpp`。同进程使用时先 `loadStaticMap()`，周期调用 `advanceTime(ROS观测时钟当前值, steady接收时钟当前值)`，再 `start()`；输入点云必须已处于 map。应从 `getDerivedMaps()` 取得一次 `shared_ptr<const DerivedMaps>`，整次规划和 SFC 共用它。`stop()` / 析构 join Worker。

Core 时钟由适配层显式推进，没有把两种时钟混成一个时间域。独立 Worker 合并待处理点云，最多保留最新待处理帧；积累帧数和点数有上限。每次实际发布都进行一次完整 Precise EDT；无障碍输出 +inf 距离。Worker 的异常通过日志、`getWorkerError()` 及 Health 显示，不静默声明地图正常。

`CostmapClient` 为 ROS 消费适配器，内部 `Costmap2D` 已改为不可变浮点地图视图。`getCost()` 不再返回 0–255；硬碰撞为 +inf，`rawObstacle()` 检查未膨胀融合障碍。`isCurrent()` 是执行层可配置的来源准入策略，MapServer 不决定运动。

远程基线没有 YAstar，本次使用独立 C++17 `grid::YAstarCore` 实现浮点输入 A* 边界；没有声称已直接移植 TDT 上游 YAstar。其边权为米制边长×两端连续代价均值；+inf 不可进入，对角线必须同时通过相邻正交格。SFC 将同版本 CollisionMask 翻转为 0=障碍、255=自由，不二次按机器人半径膨胀。拓扑图搜索、Connector、轨迹优化和控制均留在 MapServer 外。

## 合成回放与基准

构建后可运行 `src/map_server/test/test_ros_integration.py --node-binary <实际构建的map_server_node路径> --report <结果JSON路径>`。它创建临时确定 PGM/PCD、静态 TF、20 Hz 点云和 1 Hz 雷达，验证消息一致性、远处雷达、失效与恢复及退出。使用隔离的 `ROS_DOMAIN_ID`，日志路径设到可写目录。

`benchmark_map_server width height iterations` 运行完整派生与合成 PCD/点云阶段的 CPU 基准，输出 P50/P95/最大值与进程峰值 RSS。它不是实车端到端实时性证明。实际硬件、真实时间同步、PCD/PGM 叠图、机器人包络和长时间运行仍需验收。

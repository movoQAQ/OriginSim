# OriginSim nav — MapServer 重构与验收报告

日期：2026-10-22（Asia/Shanghai）。基于用户提供的 MapServer 设计文档实施；遵照用户要求，完整 Core / ROS / 下游 / 启动框架形成后才开始构建、测试。

**结果：已形成并接入完整代码框架，全仓 18 个包构建通过；36 项 C++ 测试、16 项合成 ROS 回放检查通过。** 实际雷达上游、缺失地图资源及实车验证尚未完成，不能据此认定实车可直接启动或端到端满足 20 Hz。

基线：OriginSim 远程 `nav`，提交 `c905df17ff5827629db680833a7e931e616dc6aa`；原拓扑规划器来源提交 `55a4f87c1fc1605ed24c23d402b2277a819eecb4`。拓扑规划器现作为普通 ROS 2 功能包直接纳入主仓库，保留上述提交号用于追溯来源；clone 即可获得完整源码。

## 1. 实际改动与偏差

| 位置 | 改动 |
|---|---|
| `src/map_server/include/map_server/` | 新建 GridGeometry / Grid / StaticMap / RadarTargetCache / DerivedMaps / MapServer 等纯 C++ 接口；保留并迁移点云过滤和记忆算法 |
| `src/map_server/src/` | 严格静态层加载、全 map 点云差分、独立雷达缓存、保守融合、单次完整 Precise EDT、浮点软代价与圆形硬碰撞、可停止 Worker |
| `src/map_server/ros/` | 采样时刻 TF 的 Cloud adapter、己敌 Radar adapter、单一 MapServer 节点；独立健康和轻量动态障碍输出 |
| `src/map_server/msg/` | DerivedMap / DynamicHealth / RadarTarget(s) / DynamicObstacles；浮点数据传输与生产实例标识 |
| `src/map_server/CMakeLists.txt` | Core / ROS 两个 target；Core-only 模式；核心库独立输出名避免与系统 Nav2 同名库冲突 |
| `src/navigation/topology_global_planner/` | 移除 byte NavFn 消费链，接入 C++17 float-input `YAstarCore`；保留现有拓扑搜索与分段；提供只读 Region 查询绑定 |
| `src/navigation/minimum_jerk_smoother/src/smoother.cpp` | 同一 DerivedMaps 的 CollisionMask 反转给 SFC，去除重复车体膨胀 |
| `src/navigation/minco_controller/` | 连续软代价转换；物理轮廓检查直接使用未膨胀融合障碍 |
| `src/nav_executor/` | 单一 MapClient，规划和平滑固定持有同一版本；轻量障碍接入路线检查；同步客户端配置与时钟 |
| `src/bringup/` | 单一 MapServer 启动、参数和 RViz；移除不存在的 mppi_controller 依赖 |
| `src/localization/Point-LIO-grid_map_ros2/CMakeLists.txt` | 仅调整缺失 PCD 的安装规则：存在时安装、缺失时告警；没有创建占位地图或改状态估计 / TF |
| `docs/` 与 `src/map_server/test/` | 迁移表、使用契约、回归测试、合成回放及 CPU 基准 |

旧 `static_map_node` / `dynamic_map_node` / `costmap_server_node`、Global/Local 两套生产链、旧静态/动态/膨胀 Layer 和 `uint8 Costmap.msg` 已移除；新的调试 OccupancyGrid 不作为规划输入。

必要偏差与边界：

- 远程基线没有 YAstar，只有 NavFn。本次 `YAstarCore` 是独立实现的浮点地图 A* 消费核心，**不是 TDT 上游 YAstar 的直接移植**。+inf 禁入，八邻域禁止穿角，边权为米制边长乘端点代价均值。
- 没有 Region ID 栅格时 Core 返回 INVALID_REGION；现有拓扑多边形仍保留。提供正确栅格后，以 `nav_executor.use_region_raster=true` 绑定 `regionAt()`。Region/Connector 图仍属于规划器。
- 默认 Bringup 配置引用的 **`topo26.yaml` 未随远程分支提供**。不把现有 `topo25.yaml` 冒充 2026 地图拓扑；需提供真实匹配文件。

## 2. 数据与时序契约

| 输入 | Topic / 类型 | 时间 / ID | 默认失效策略 |
|---|---|---|---|
| 点云 | 可配 `/cloud_registered`，sensor_msgs/PointCloud2 | 采样 stamp 非零，float32 XYZ；每帧按该时刻 TF 转 map；独立 steady 接收时间 | 输入 .5 s；Cloud 记忆最长 1 s；4 帧 / .35 s 积累；定位跳变 .5 m / .35 rad 时仅重置 Cloud |
| 己方雷达 | 可配 `/radar/allies`，新增 map_server/RadarTargets | map XY、ROS 观测时间、稳定 uint16 ID；本车 ID 排除 | 来源 stale 2.5 s，半衰期 2 s，目标最长 4 s，最低置信度 .25 |
| 敌方雷达 | 可配 `/radar/enemies`，同上 | 独立来源时间与完整列表；下游与己方同样视为硬障碍圆 | 同上，互不删除或续期 |

实际雷达硬件接口没有接通：当前消息是明确新增的接入协议。缺少设备采样时间时，上游须显式使用 ROS 接收时刻并标记 time_degraded；无稳定 ID 时采用每来源完整列表替换，身份过滤未验证。空列表撤销该来源，断流按年龄衰减并报告 STALE。时钟回退清空所有动态来源。静态 PCD 始终处于 map，不随 odom 搬移。

默认 `radar.self_id=-1`，本车过滤未验证。默认客户端 `require_cloud=true`，`require_radar=false`；这是执行层可配置准入策略。Worker 错误通过 Health / 日志报告，不把地图继续输出解释为可以继续运动。

## 3. 地图消费边界

- Core：`loadStaticMap()`、`updateRegisteredCloud()`、己敌更新、`regionAt()` / `heightAt()` / `depthAt()`、`getDerivedMaps()`、`getLatestDynamicObstacles()`、`getDynamicHealth()`、start/stop；C++ 调用方通过 `advanceTime()` 独立推进 ROS 与 steady 两个时钟。
- 静态 Occupancy / Region / Height / Depth 严格校验同一几何；只支持轴对齐；未知占据拒绝。Region 与 Height/Depth 不影响 EDT / Costmap。
- 每次实际发布使用同一融合输入的一次完整非负 EDT，距离为 float32 米，硬碰撞为 +inf；无障碍距离明确为 +inf。没有 Signed ESDF 或增量 EDT。
- 机器人默认半径 .43 m，加 .05 m 裕量，并保守计入栅格离散误差与地图外边界；需要实车标定。
- `version` 为障碍派生版本，`input_version` 为应用的来源状态版本；健康心跳独立，未因健康变化伪造地图版本。`producer_id` 区分 ROS 生产进程重启，客户端拒绝跨实例地图 / 心跳组合。
- YAstarCore 与 SFC 固定引用同一个 shared_ptr<const DerivedMaps>；SFC 将 CollisionMask 反转为 0=障碍、255=自由，不再次执行机器人半径膨胀。
- 地图计算不占用输入锁；Worker 忙时合并到最新待处理点云，不无限排队；停止时 join。

## 4. 构建、测试与运行验证

环境：Ubuntu 24.04.3 LTS，ROS 2 Jazzy，GCC 13.3.0，C++17，CPU-only。

| 验证 | 结果 |
|---|---|
| 全仓 colcon build | **18 packages finished**，无失败；缺失 PCD 明确告警 |
| 最后增量构建 | map_server / nav_executor / bringup 3 个包通过 |
| Core 契约 | **30 项通过**：几何、静态语义、Region 位深、Height 查询、来源独立、雷达超时/空列表/本车 ID、保守重采样、EDT、软代价、溢出、记忆、最新帧支持、定位跳变、非有限点云、版本一致性、停止、A* 穿角及连续代价 |
| ROS adapter | **5 项通过**：按采样时间变换、禁止最新 TF 替代、布局/预算拒绝、Radar 分源/时间与坐标校验 |
| MapClient | **1 项通过**：实例重启低版本接受、跨实例心跳拒绝、非法版本拒绝与恢复 |
| colcon test-result | **3 个 CTest 注册项，0 errors / 0 failures / 0 skipped**；合计上述 36 项 GTest 用例 |
| 无 ROS Core-only build + CTest | 通过；ldd 中无 rclcpp / tf2_ros / Nav2 / rosidl 依赖 |
| 合成 ROS 回放 | **16 项通过**，20 Hz 点云 / 1 Hz 雷达；包含 8 m ROI 外雷达、本车排除、过期/恢复、空列表、TF 缺失、同版本消息一致性、线程退出 |
| 导航执行器冒烟 | 使用仓库现有 topo25.yaml 配置，可初始化进入 IDLE 并正常退出；未发送运动目标 |
| 默认 Bringup 冒烟 | 明确失败于缺失 topo26.yaml；这是待提供的真实资源，没有用别的赛场拓扑替代 |
| 结构检查 | Python 语法、YAML、package.xml、主仓库与子模块 diff 空白检查通过；补丁应用检查另附 |

执行顺序满足“完整框架形成后再测试”。没有执行实车、真实 bag / Gazebo 全导航回放或全部未修改包的测试。ROS 回放在本机共享内存通信下通过；沙箱限制 UDP socket，未验证跨机 UDP 传输。

主要实际命令（相对于本次聊天根目录）：

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base work/log build \
  --base-paths work/OriginSim/src --build-base work/build \
  --install-base work/install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source work/install/setup.bash
ROS_LOG_DIR="$PWD/work/ros-log" ROS_DOMAIN_ID=87 \
  colcon --log-base work/log test --base-paths work/OriginSim/src \
  --packages-select map_server --build-base work/build --install-base work/install \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base work/build/map_server --verbose
cmake -S work/OriginSim/src/map_server -B work/core-only-build \
  -DMAP_SERVER_CORE_ONLY=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build work/core-only-build -j2
ctest --test-dir work/core-only-build --output-on-failure
```

初次配置平滑器复用了本地固定版本 OSQP/OsqpEigen/QDLDL 源码缓存，使用 CMake 的 FETCHCONTENT_SOURCE_DIR_JERK_OSQP / JERK_EIGEN / QDLDL 覆盖；后续构建沿用缓存。缓存不包含在源码仓库；联网环境按原 CMake 固定版本下载，离线环境可通过上述 CMake 参数指向实际缓存目录。

完整日志与原始结果见交付目录 `validation/`。

## 5. 性能实测

CPU：AMD Ryzen 7 H 255，8 核 16 线程；Release，CPU-only。每组预热 5 次、计量 100 次。基准使用可复现的**合成障碍图**与雷达圆；583×300 与仓库地图尺寸一致，但不是实车数据或 PCD/PGM 对齐验证。

### 583×300，0.05 m/格

| 阶段 | P50 ms | P95 ms | 最大 ms |
|---|---:|---:|---:|
| fusion | 0.009 | 0.032 | 0.091 |
| edt | 1.165 | 1.625 | 1.787 |
| costmap | 0.837 | 0.945 | 0.996 |
| derived_total | 2.005 | 2.519 | 2.821 |
| cloud_prepare_11000_points | 0.366 | 0.469 | 0.862 |
| pcd_difference_filters_10000_reference_points | 12.393 | 13.156 | 13.611 |

峰值进程 RSS：76796 KiB（包含测试程序及依赖，非机器人整栈内存）。

### 1000×1000，0.05 m/格

| 阶段 | P50 ms | P95 ms | 最大 ms |
|---|---:|---:|---:|
| fusion | 0.088 | 0.215 | 0.698 |
| edt | 9.311 | 10.157 | 10.678 |
| costmap | 4.916 | 5.445 | 5.763 |
| derived_total | 14.479 | 15.418 | 15.844 |
| cloud_prepare_11000_points | 0.295 | 0.432 | 0.815 |
| pcd_difference_filters_10000_reference_points | 12.148 | 13.002 | 13.398 |

峰值进程 RSS：82068 KiB。

点云阶段为独立合成基准：11000 个输入点、10000 个静态 PCD 参考点，局部 320×320 / .05 m 栅格；PCD 差分项含 KD-Tree 查询、SOR、密度、连通域和最新帧支持验证，不包含真实 TF / 运动补偿或四帧实际传感器累积。

合成 ROS 回放收到 37 个派生版本，消息收到时 `generated_at_s` 年龄 P50=51.28 ms、P95=52.56 ms、最大=108.04 ms。此字段基于 Worker 使用的 ROS 时间快照，受 20 Hz 时钟推进及发布调度粒度影响；**不是传感器到规划器的端到端延迟**。未测真实传感器端到端 P50/P95、持续 20 Hz 满负载或单独指针交换耗时，不能相加独立阶段并宣称端到端实时性达标。

## 6. 遗留风险与需要的实际数据

1. 真实雷达发布器、ROS 消息映射、时间域同步、本车 ID 与目标物理半径尚待接通及核对。
2. 静态 PCD、Region uint16 图及可选 Height/Depth 未随分支提供；没有制作占位或推测数据。
3. `topo26.yaml` 缺失；2026 地图/PCD/拓扑必须实际配套并叠图确认。现有 topo25 仅做初始化验证。
4. Point-LIO 实际运行仍需要其配置对应 PCD；仅安装不再因资源缺失而失败。
5. 采样时间 TF、真实 map→odom 跳变阈值、CPU 过载及长时间失效/恢复需在实际定位与传感器上验证。
6. .43 m 机器人圆形包络、.05 m 裕量、雷达衰减及不确定性参数均为调试起点，未完成实车轮廓标定。
7. 本次没有改写拓扑 Connector 约束策略、路线监控业务策略或定位状态估计；未验证真实目标行驶与整车闭环。

## 7. GitHub 仓库交付

本次追加要求将 OriginSim fork 后推送，并将 `src/navigation/topology_global_planner` 转为普通功能包。主仓库直接跟踪拓扑包内的源码、配置、CMakeLists.txt 与 package.xml；删除 `.gitmodules` 和原 gitlink，不再需要初始化子模块。

此前两份补丁和压缩包属于首次本地交付。GitHub 版本以主仓库完整提交为准，不再分别应用拓扑子模块补丁，也不要执行旧的 submodule 更新来覆盖普通包。

初次验证记录和性能限制仍适用；转换后已重新验证：colcon 正确识别普通 topology_global_planner 包，6 个相关 ROS 包增量构建通过，36 项 GTest（3 个 CTest 注册项）全部通过，Git 索引无 160000 gitlink，拓扑源码由主仓库直接跟踪。Fork 为 `movoQAQ/OriginSim`，基线和修改位于 `nav` 分支。

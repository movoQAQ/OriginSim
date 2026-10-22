# MapServer 与不可变地图视图

工作空间为 `/home/lyu/hxy/nav`，Gazebo 为 `/home/lyu/hxy/gazebo`。基于 OriginSim 远端 nav 分支的 MapServer 框架接入 hxy 的传感器、定位与仿真资源。`topology_global_planner` 位于 `src/navigation/`，是普通 ROS 2 功能包。

## 数据与接口

| 通道 | 内容 | 发布条件 |
| --- | --- | --- |
| `/map_server/static` / `StaticMap` | 占据、区域、可选高度与深度 | 启动、时钟回退后的新 producer；transient local |
| `/map_server/derived` / `DerivedMap` | 融合障碍、距离场、浮点代价、碰撞掩码 | 融合障碍改变 |
| `/map_server/dynamic` / `DynamicObstacles` | 最新云障碍、统一圆形障碍 | 障碍几何改变 |
| `/map_server/status` / `MapStatus` | producer、已发布版本、来源健康、Worker 错误 | 20 Hz 轻量心跳 |
| `/map` | 静态 OccupancyGrid，供 RViz 显示 | 静态地图发布时 |
| `/map_server/costmap` | RViz 用的压缩代价图 | 有订阅者且派生版本改变 |

雷达输入暂用 `/radar/allies` 和 `/radar/enemies` 的 `RadarTargets`；输入适配器维护来源、ID、自身过滤、有效期。下游只收到位置和有效半径，不重复传输输入目标协议。未来外部位段协议应在输入适配器解析。

Core 的 `getStaticMap()` 返回同一份只读静态地图。ROS 节点直接使用它，不再次读取 PGM、区域或标量图。静态参考 PCD 由点云过滤器加载；它与占据栅格用途不同。PGM 未知像素在二值规划图中保守作为障碍。

ROS 消费端只有一个 `MapClient`，调用 `auto view = maps->view()` 捕获 `shared_ptr<const MapView>`。读取前用 `maps->isCurrent(*view)` 校验生产者、地图版本、动态版本、健康状态和 ROS/steady 心跳年龄。回调校验输入、移动消息数组到不可变数据，只在发布新视图指针时短暂加锁。调用方无公开锁、可写地图或旧接口适配层。路线阻塞确认使用状态中的 input_version；稳定障碍不需要反复发送几何数组，也能依据新观测累计确认。

`MapView` 提供 `geometry()`、`derivedMaps()`、`staticMap()`、`dynamicObstacles()`、`cost()`、`collision()`、`latestOccupied()`、`regionAt()`、`heightAt()` 和 `depthAt()`。旧视图在新消息到达后仍引用原有数据。规划与平滑共享一次捕获的视图；控制器在一次求解及所有预测碰撞检查中使用同一视图。派生图的 cost 为浮点，硬碰撞为正无穷；`latestOccupied()` 使用静态占据和最新动态几何，允许已经撤回的动态障碍及时消失。

## hxy 仿真资源

默认 startup 使用 `bringup/map/rmuc_2027.yaml`（PGM）和 `bringup/map/rmuc_2027.pcd`。`depth_yaml` 保持空字符串，静态消息的 depth 数组为空，查询返回无值；不生成替代 Depth。

当前保存的 `topo.yaml` 区域坐标未验证与 rmuc_2027 对齐，因此配置显式 `use_topology: false`。先验证栅格搜索、平滑和控制；提供匹配区域/拓扑后再启用。区域图、高度图也未配置。

```bash
cd /home/lyu/hxy/nav
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch bringup startup.launch.py
# 另一终端，source 同一工作空间后：
ros2 launch nav_executor navigation_launch.py
```

工作区根目录 `start.sh` 同时启动 Gazebo 与导航，要求 NVIDIA 驱动和可用图形会话。`startup.launch.py rviz:=false` 可关闭导航 RViz。

## 验证

```bash
ctest --test-dir build/map_server --output-on-failure
python3 src/map_server/test/test_ros_integration.py \
  --node-binary install/map_server/lib/map_server/map_server_node \
  --report /home/lyu/hxy/.task-work/ros-integration.json
```

ROS 回放同样使用 hxy 的 rmuc_2027 PCD/PGM，Depth 为空；输入点云和雷达为测试消息。真实 Gazebo 验证须另外检查定位、来源健康、规划和速度闭环，不能由回放结果代替。

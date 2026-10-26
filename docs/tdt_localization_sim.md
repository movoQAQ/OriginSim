# T-DT 定位后端接入 OriginSim

`nav` 分支中的 `bringup/tdt_sim.launch.py` 使用原 Point-LIO 作为里程计前端，接入
[Z-Localization](https://github.com/mozihe/z-localization) 的 `backend` 包。Gazebo 保持
`movoQAQ/OriginSim` 的 `gazebo` 分支。两个分支分别构建、逐层 source；不要把两套
`rmu_gazebo_simulator` 或 `bringup` 源码放进同一个工作区。
编写时基线为 `nav` e94ed05f681f、`gazebo` 3c52828f203a、Z-Localization
76e126328b67。

## 接口和坐标系

| 来源 | 后端输入 | 坐标系 |
| --- | --- | --- |
| Point-LIO `/odom` | 高频与扫描末端里程计 | `odom → aft_mapped` |
| Point-LIO `/cloud_registered` | 已去畸变点云 | `odom`，与扫描末端里程计同时间戳 |
| T-DT 地图包 | 先验全局地图 | `map`，使用未改动的 `rmuc_2026.pcd` |

`tdt_backend.yaml` 选择 `legacy` 输入，关闭 UWB 和旧兼容输出。Point-LIO 保留
`odom → aft_mapped`；只有 T-DT 后端发布 `map → odom`。本入口没有启动原有的
`landmark_alignment` 或静态 `map → odom`。不要并行启动 `startup.launch.py`、
`no_smoother_tunnels.launch.py` 或 `tf.launch.py`。
仿真默认出生点对应雷达在 `map` 中约 `(3.4, 9.5, 0.88)`、航向 `-62.4°`。
车体出生航向接近 `0°`，但 Gazebo 模型的 `chassis → livox_frame` 静态外参含
`-62.45°` 航向，后端使用的是传感器位姿；
`startup_hint.force_until_matched` 禁止在该提示验收前改用全局描述子候选。
换出生点时需同步传 `initial_x`、`initial_y`、`initial_z`、`initial_yaw_deg`。

## 构建与启动

以下命令使用 ROS 2 Jazzy（与 `gazebo` 分支一致），假设三个仓库分别位于
`~/OriginSim-gazebo`、`~/OriginSim` 和 `~/z-localization`。Z-Localization 上游文档
标注 Humble，跨 Jazzy 的构建和运行需要实际验证。

先安装后端的系统依赖（至少 ROS Jazzy 的 `ros-jazzy-gtsam` 4.2、PCL、
nanoflann、TBB 和 MPI 开发包），以及导航依赖 `ros-jazzy-grid-map-core`、
`ros-jazzy-grid-map-msgs`、`ros-jazzy-grid-map-ros`；其余 ROS 依赖按各包的
`package.xml` 安装。Ubuntu 24.04 的 `libgtsam-dev` 4.2 在此环境中
导出缺失的 `libCppUnitLite.a`，需要明确选择 ROS Jazzy 的 GTSAM：

```bash
# Gazebo 工作区，已在此分支验证的包
cd ~/OriginSim-gazebo
colcon build --symlink-install --packages-select \
  rmoss_interfaces rmoss_gz_resources rmoss_gz_plugins rmoss_gz_base \
  sdformat_tools pb2025_robot_description ign_sim_pointcloud_tool rmu_gazebo_simulator

# T-DT 后端及消息包；不编译它附带的 Livox 驱动
source /opt/ros/jazzy/setup.bash
source ~/OriginSim-gazebo/install/setup.bash
git -C ~/z-localization apply ~/OriginSim/tools/z-localization-jazzy.patch
cd ~/z-localization
colcon build --symlink-install --packages-up-to backend \
  --cmake-args -DGTSAM_DIR=/opt/ros/jazzy/lib/cmake/GTSAM
source install/setup.bash

# 导航工作区，在 Gazebo 和 T-DT 安装层之上构建
cd ~/OriginSim
colcon build --symlink-install --packages-up-to bringup
source install/setup.bash

# 从 Gazebo 静态 PCD 生成 T-DT 地图包；不会改动原 PCD 或 PGM
~/OriginSim/tools/build_tdt_prior.sh
```

先运行仿真，再在第二终端按同样顺序 source 三层工作区后启动定位：

```bash
ros2 launch rmu_gazebo_simulator tunnel_sim.launch.py

ros2 launch bringup tdt_sim.launch.py \
  prior_map:=$HOME/OriginSim/src/bringup/map/tdt_prior
```

定位达到可信状态后，可重启该入口并加 `navigation:=true`，同时启动 `nav_executor`；
默认关闭导航，便于先核对定位与 TF。`params_file` 可选择导航参数文件。
从默认出生点验证小段导航，可在 `navigation:=true` 时另开终端发布目标：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 4.2, y: 9.5}, orientation: {w: 1.0}}}'
```

Gazebo 分支的仿真需要提供 `/velodyne_points`、`/livox/imu`、`/clock`；
Point-LIO 应产生 `/odom`、`/cloud_registered`。生成先验地图时，
`--sensor-z`/`--sc-lidar-height` 暂按 0.88 m 的雷达高度设置，须用实际仿真传感器
外参核对。Z-Localization 的定位状态必须进入 `NORMAL` 或 `OBS_DEGRADED` 且
`pose_trusted=true`，才允许把全局位姿给导航控制。

## 仿真验收

Gazebo 真值仅用于离线验收，不送进定位后端。另开终端 source 工作区后运行：

```bash
python3 ~/OriginSim/tools/verify_tdt_sim.py --truth-topic /Odometry --seconds 60
```

脚本通过 TF 把 Gazebo 的车体真值换算到实际 IMU/雷达 `livox_frame`；
Point-LIO 的里程计消息把该估计标为 `aft_mapped`，验收时按物理传感器系比较。
如果桥接消息缺少 `child_frame_id`，追加 `--truth-frame chassis`。
脚本要求至少 20 个时间差不超过 0.15 s 的可信样本；位置误差中位数不得超过
0.75 m、航向误差中位数不得超过 0.35 rad，95 分位分别不得超过 1.5 m 与
0.7 rad，可信样本占比至少 90%，且验收结束时仍须 `pose_trusted=true`。
静止启动后还应驱动机器人经过有不同
几何结构的位置，再重复验收。对定位状态、`map → odom` TF、点云及先验地图包的
首次联机验证结果需要记录在这里，不能把静态检查当成仿真通过。

Jazzy 容器内的后端、消息包、Gazebo 及导航依赖已构建成功。先验地图工具处理了
1,364,092 个原始 PCD 点，生成 220 个扫描关键帧，平均每帧 8,607 个点。
首次无提示且对应比例下限为 0.30 的尝试错误匹配到约 `(25.5, 6.6)`；
Gazebo 真值约为 `(3.4, 9.5)`。因此增加了出生点位置和传感器航向提示，并限制
首次匹配修正不超过 2 m。按此配置重启后，后端匹配先验关键帧 163 并进入 `NORMAL`；
静止 35 秒验收得到 1530 个可信同步样本，位置误差中位数 0.009 m、航向误差中位数
0.004 rad。`map → odom` TF 已通过 tf2 查询确认。

手动发送短时 `/cmd_vel_chassis` 转向、前进后，Gazebo 车体从约 `(3.40, 9.50)`
移动到 `(4.23, 10.35)`；覆盖运动的 42 秒验收得到 1916 个同步可信样本，位置误差
中位数 0.020 m、航向误差中位数 0.011 rad。运动中曾短暂进入 `OBS_DEGRADED`，
之后恢复 `NORMAL`。从新位置以相应的传感器位姿提示重启，30 秒静止验收得到
1364 个可信样本，位置误差中位数/95 分位为 0.083/0.084 m，航向均为 0.012 rad。

重新从安全出生点启动 `navigation:=true` 后，向 `/goal_pose` 发布 `map` 坐标
`(4.2, 9.5)`。导航执行器规划并执行路径，Gazebo 车体最终停在约
`(4.136, 9.502)`，距离目标 0.064 m。覆盖导航的 45 秒验收得到 1935 个可信样本，
真值位移 0.736 m；位置误差中位数/95 分位为 0.007/0.041 m，航向为
0.005/0.006 rad。期间出现一次短暂 `SUSPECT`，导航因 TF 超过 0.5 s 时限停下，
恢复定位后自动重新规划并继续行驶。当前位置旁的墙体使另一组导航目标的起点
落入占据图安全膨胀区，规划器明确拒绝该目标；应先确认出生点及目标具备车体
半径所需的可通行空间。当前地图服务器按 `Static-only navigation` 工作，动态
障碍物通路未纳入本次验收。

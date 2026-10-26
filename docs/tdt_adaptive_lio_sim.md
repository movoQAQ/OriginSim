# Adaptive-LIO + T-DT 仿真接入与验收

本入口使用 `movoQAQ/OriginSim` 的 `nav` 分支、独立的 `gazebo` 分支、
`mozihe/z-localization` 的 T-DT 后端，以及
`ECUST-Origin-Sentry-Team/Adaptive-LIO` 的 `ljb` 分支。验证基线分别为
`e94ed05`、`3c52828`、`76e1263`、`97a3b0e`。Gazebo 真值只进入验收脚本，
没有接入前端或后端。

## 数据链路

| 输入或输出 | 话题 / 坐标系 | 用途 |
| --- | --- | --- |
| Gazebo GPU LiDAR | `/velodyne_points`，`livox_frame` | Adaptive-LIO 输入；点 `time=0` 表示瞬时扫描 |
| Gazebo IMU | `/livox/imu`，SI 单位 | Adaptive-LIO 输入；200 Hz |
| Adaptive-LIO 里程计 | `/odom`，`odom → aft_mapped` | T-DT `legacy` 里程计输入 |
| Adaptive-LIO 注册点云 | `/livox/scan`，`odom` | T-DT `legacy` 点云输入 |
| PCD 先验地图包 | `map/tdt_prior` | T-DT 全局激光修正，220 个关键帧 |
| T-DT 全局 TF | `map → odom` | 导航使用；后端唯一发布者 |

`tdt_adaptive_backend.yaml` 启用 PCD 先验地图、关闭 UWB 输入。因而这次结果说明
LiDAR 先验地图定位可行，不验证 UWB。先验地图使用原 `rmuc_2026.pcd` 生成，
Gazebo 分支及其地图素材未改动。仿真 IMU 与激光相差 3 cm，
`mapping.extrinsic_T=[0,0,0.03]`；加速度已经是 m/s²，`gnorm=1.0`。
Adaptive-LIO 的 `aft_mapped → chassis_est` 外参对应 Gazebo 的物理传感器安装，
导航的 `fake_baselink` 使用 `chassis_est`，不使用 Gazebo 真值 `chassis`。

Gazebo 的激光逐点时间全为零，原 Adaptive-LIO Velodyne 转换会得到零扫描时长。
补丁把这种扫描的所有点标为同一时刻，并给同步器 0.1 ms 的有限窗口。
激光订阅使用 best effort 传感器 QoS；IMU 桥接使用 reliable，故保持 reliable
订阅。仿真配置把输入点抽样设为每 3 点取 1 点，把发布点云的体素尺寸设为 0.05 m。
这消除了本场景中 PCL 的体素网格索引溢出警告，同时后端仍能匹配先验地图。

在导航负载下，原前端的 50 ms IMU 缺口门限曾使其主动停机。补丁保留
50 ms 默认值，仿真配置允许最多 150 ms：对上限内缺口做不超过 25 ms 的
IMU 插值预测步，并增大过程噪声；更大缺口仍停机。
原 0.5 s 激光修正后的 IMU 预测窗口在该场景会间歇中断 `/odom` 和全局 TF。
仿真配置把窗口限于 2.0 s，导航对全局 TF 的超时也设为 2.0 s，
但里程计接收超时仍为 0.5 s。这些是本仿真负载的参数，不能直接用作实车标定。

## 构建

先按 [T-DT 接入说明](tdt_localization_sim.md) 构建 Gazebo、T-DT 及导航工作区，
并运行 `tools/build_tdt_prior.sh`。另外安装 `libceres-dev`，在单独的
Adaptive-LIO 工作区构建其 `adaptive_lio` 与 `livox_ros_driver2`：

```bash
git clone --branch ljb https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO.git ~/Adaptive-LIO
git -C ~/Adaptive-LIO apply ~/OriginSim/tools/adaptive-lio-sim.patch
source /opt/ros/jazzy/setup.bash
cd ~/Adaptive-LIO
colcon build --symlink-install --packages-up-to adaptive_lio --cmake-args -DBUILD_TESTING=OFF
```

运行时依次 source `/opt/ros/jazzy/setup.bash`、Gazebo、Z-Localization、
Adaptive-LIO、OriginSim 的 `install/setup.bash`。在两个终端分别启动：

```bash
ros2 launch rmu_gazebo_simulator tunnel_sim.launch.py
ros2 launch bringup tdt_adaptive_lio_sim.launch.py \
  prior_map:=$HOME/OriginSim/src/bringup/map/tdt_prior navigation:=true
```

默认初始提示对应 Gazebo 出生点 `(3.4,9.5)`，传感器航向 `-62.4°`。
Gazebo 车体若在无命令时自转，先用 `tools/drive_tdt_sim.py --speed 0
--yaw-rate 0 --seconds 2` 持续发送零速度，再核对 `/Odometry`，必要时复位机器人。
从其他位置启动时，同步修改启动参数 `initial_x`、`initial_y`、
`initial_yaw_deg`；传感器航向约等于真值车体航向减 62.4°。
不要同时启动原来的 `startup.launch.py` 或其他 `map → odom` 发布者。

## 验收结果

使用同一 Gazebo 场景、同一 PCD 先验地图及 `tools/verify_tdt_sim.py`。
最终配置从真值约 `(3.542,9.517)` 启动，导航目标为 `map` 中的 `(4.2,9.5)`。
前端 IMU 初始化成功，后端首次同步帧含 2,966 点，按启动提示匹配关键帧 121，
进入 `NORMAL`。导航规划并执行 17 个位姿的路径；70 秒真值验收结果：

| 指标 | 结果 |
| --- | ---: |
| 可信同步样本 | 12,427 |
| 车体位移 | 0.749 m |
| XY 误差中位数 / 95 分位 | 0.004 / 0.012 m |
| 航向误差中位数 / 95 分位 | 0.006 / 0.006 rad |
| 可信样本比例 | 99.9% |
| 结束状态 | `NORMAL`，`pose_trusted=true`，`tf_published=true` |
| 终点距导航目标 | 0.093 m |

本轮日志没有 IMU 故障或导航路径执行停止；前端每 5 秒的高频里程计检查
均为 `valid=1`。Adaptive-LIO 的核心、几何和 IMU 质量测试也通过。
验收脚本现在要求结束时 `tf_published=true`，避免把只有后端可信标志、
却没有导航所需 TF 的场景算作通过。

测试覆盖了约 0.75 m 的短路径与仿真的瞬时 GPU LiDAR；
没有验证实车 Livox 逐点时间、长距离路径、全速导航或 UWB 融合。

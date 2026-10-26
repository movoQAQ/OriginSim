# T-DT + ZLIO 仿真入口

`nav` 分支的 `bringup/tdt_zlio_sim.launch.py` 使用
[z-localization](https://github.com/mozihe/z-localization) 自带的 `frontend/zlio_node` 和
`backend/backend_node`。Gazebo 仍使用 OriginSim 的 `gazebo` 分支。此入口不启动
Point-LIO；ZLIO 的 `/frontend/frame`、`/frontend/odometry`、`/frontend/session`
直接连接后端，只有后端发布 `map → odom`。

## 输入和坐标

Gazebo 的 `/livox/imu` 加速度约为 9.8 m/s²，ZLIO 使用倍率 `1.0`。
原始 `/livox/lidar/pointcloud` 没有逐点时间字段，现有 Gazebo 转换器输出的
`/velodyne_points` 带 `time=0`；Gazebo GPU LiDAR 是瞬时扫描，因此 ZLIO 按同一
时刻接收该帧。`zlio_cloud_adapter.py` 每隔 12 个回波保留一个，从 23,040 点降到
1,920 点，并原样保留坐标、时间、方位覆盖及消息时间戳，防止前端在启动优化时
耗尽输入队列。启动窗口从 20 帧缩到 8 帧，采样体素设为 0.2 m，最大交替细化
次数设为 6；这些调整仅用于当前 CPU 仿真。真实逐点扫描的去畸变能力不能由此
工况验证。

仿真 IMU 的角速度噪声很小。入口将 ZLIO 运行滤波的
`filter.gyro_variance` 从硬件配置的 `0.25` 降到 `0.001 rad²/s²`，使转向时的
里程计跟随实测角速度。这个值只针对当前 Gazebo IMU；实机需要按传感器噪声
重新标定。

Gazebo SDF 中 LiDAR 比 IMU 高 0.03 m。启动入口从安装的 `frontend/config/lio.yaml`
生成临时配置，将 `sensor.body_from_lidar` 改为平移 `(0, 0, 0.03)`、旋转单位
四元数，并应用上面的初始化及陀螺仪参数。后端的 `imu` 位姿对应物理
`livox_frame`；默认出生提示约为
`(3.4, 9.5, 0.88)`、航向 `-62.4°`。该航向包含模型中的传感器固定安装角。
导航使用 ZLIO 的 `odom → imu` 和静态 `imu → chassis_est`，避开 Gazebo 真值
`odom → chassis`。不同出生点应同时指定 `initial_x`、`initial_y`、`initial_z`、
`initial_yaw_deg`。

## 构建与运行

先按 [Point-LIO 版构建步骤](tdt_localization_sim.md)准备 ROS 2 Jazzy、Gazebo、
后端和先验地图。`tools/z-localization-jazzy.patch` 还给仓库自带的
`livox_ros_driver2` 添加 `LIVOX_MESSAGES_ONLY` 选项；仿真只需要其 ROS 消息类型，
无需硬件 SDK。在 Z-Localization 工作区执行：

```bash
git apply ~/OriginSim/tools/z-localization-jazzy.patch
source /opt/ros/jazzy/setup.bash
cd ~/z-localization
colcon build --symlink-install --packages-up-to frontend \
  --cmake-args -DLIVOX_MESSAGES_ONLY=ON -DCMAKE_BUILD_TYPE=Release \
  -DLOCALIZATION_WARNINGS_AS_ERRORS=OFF -DBUILD_TESTING=OFF
source install/setup.bash

cd ~/OriginSim
colcon build --symlink-install --packages-select bringup
```

按 `/opt/ros/jazzy`、`OriginSim-gazebo/install`、`OriginSim/install`、
`z-localization/install` 顺序 source 后，在两个终端启动：

```bash
ros2 launch rmu_gazebo_simulator tunnel_sim.launch.py
ros2 launch bringup tdt_zlio_sim.launch.py \
  prior_map:=$HOME/OriginSim/src/bringup/map/tdt_prior
```

真值验收沿用
`python3 ~/OriginSim/tools/verify_tdt_sim.py --seconds 60`。脚本使用 Gazebo
`/Odometry` 和模型静态 TF 算出实际 IMU 位姿，真值不送进 ZLIO 或后端。
需要做手动移动验收时，先运行验收脚本，再在另一个终端执行：

```bash
python3 ~/OriginSim/tools/drive_tdt_sim.py --speed 0.10 --seconds 8
python3 ~/OriginSim/tools/drive_tdt_sim.py --speed 0 --yaw-rate 0.10 --seconds 3
```

该脚本会在运动结束后持续发布 3 秒零速度。Gazebo 底盘控制器会保持最后一次
收到的速度目标，因此不能仅靠结束速度发布进程停车。

## 仿真结果

初次在未设置 `CMAKE_BUILD_TYPE` 的构建中，前端处理落后于仿真输入，先后触发
队列溢出和 IMU 历史水位过期。后端虽曾显示 `NORMAL`，真值验收却测得约
20.6 m 和 3.14 rad 的错误定位。因此本入口限制 LOST 阶段的搜索半径为 2 m，
关闭全局候选补充，并要求实际真值验收通过才判定可用。

Release 构建用同一先验地图在出生点启动，首次匹配到 KF 141 并进入 `NORMAL`。
静止 35 秒真值验收取得 6325 个样本，位置中位误差 0.019 m、95 分位 0.034 m，
航向中位和 95 分位均 0.005 rad，结束时 `pose_trusted=true`。

使用持续停车指令后，0.202 m 直线移动的 42 秒验收有 7082 个样本，位置误差
中位数 0.038 m、95 分位 0.140 m，航向误差中位数 0.003 rad、95 分位 0.005 rad，
可信占比 99.6%，结束时 `NORMAL`。后续约 0.6 m 直线移动加短转向的 65 秒验收
虽然通过阈值，但转向时曾出现 13° 前后端航向冲突，继续转向后进入 `LOST`。
这个结果来自原始 `filter.gyro_variance=0.25`，不能据此宣称持续转向可用。

降低仿真 IMU 的陀螺仪方差后，重复 6 秒、`0.10 rad/s` 的转向，Gazebo 真值
转过约 0.484 rad，ZLIO `/odom` 转过约 0.455 rad。42 秒验收有 7316 个样本，
位置误差中位数 0.029 m、95 分位 0.036 m，航向误差中位数 0.006 rad、95 分位
0.033 rad，可信占比 96.8%，结束时 `NORMAL`。期间有短暂 `SUSPECT`，并自动
恢复到 `NORMAL`；还需要更长路径和更高速度验证，不能把这一工况外推为全速导航。

`navigation:=true` 目前仅保留为联调入口，**未通过导航验收**。将导航器的
`map_client.transform_tolerance`、`nav_executor.tf_timeout` 和
`nav_executor.connector_executor.pose_timeout` 都放宽到 1.2 s 后，从出生点
向 `(4.2, 9.5)` 发布 `/goal_pose`。机器人实际移动 1.393 m，定位状态依次降为
`OBS_DEGRADED`、`SUSPECT`、`LOST`；70 秒验收位置误差 95 分位 1.920 m，
可信样本占比仅 42.0%，结束时 `pose_trusted=false`，验收脚本以非零状态退出。
导航器多次报告路径碰撞和重规划。此工况下不要把 ZLIO 后端位姿用于自主导航；
需要进一步解决连续运动时的里程计漂移及恢复问题，并重新跑完整目标点验收。

后续一次同步记录同时采集 Gazebo `/Odometry`、ZLIO `/odom`、后端位姿和底盘
指令。以启动时真值对齐 ZLIO 局部里程计后，在首次前端重置之前，第 17、20、
22 秒的 ZLIO 平面误差分别约为 0.27、0.87、1.68 m；后端同期分别约为
0.27、0.79、1.61 m，角度误差仍小于 0.03 rad，消息时间差约 0.02 s 以内。
这表明连续导航工况中前端**平移**预测先偏离真实运动，后端初期随预测偏离；
随后先验图修正与预测冲突，后端拒绝较大修正。此前每次观测的先验图匹配均已
成功载入 PCD 地图包；UWB 在配置中主动关闭。根因仍需进一步区分 ZLIO 本身
在高速运动下的跟踪、瞬时雷达及抽样输入、地图配准误关联和导航控制之间的影响。

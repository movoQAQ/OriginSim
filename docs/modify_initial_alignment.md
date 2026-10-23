# 切回 modify 与初始位姿校准（2026-10-23）

当前仿真和导航已停止，按用户要求不重启。Bringup 已从 GICP 切回 modify_map_to_odom；GICP 包保留，但不会由默认启动链加载。静态地图模式、选定 rmuc_2027.stl、原 PGM/PCD 均保留。

Gazebo 当前 rmuc_2027 出生配置为 x=3.4、y=9.5、z=0.89、yaw=0；modify 为 X=3.4、Y=9.5、Z=0.8758、Rotation=-1.095。当前场景单独定义出生位置，未改 rmuc_2025/2026 共用的原配置。

高度校准依据实际 STL：出生中心地面 z=0.2，轮半径 0.0758，chassis 到 IMU 的模型高度差 0.6，名义落稳 IMU 高度为 0.8758。四个轮位附近地面约 0.1930..0.2064，存在小坡度，因此 Gazebo 出生高度留为 0.89，使初始轮胎高于最高采样表面约 0.0078 m，随后落稳。start.sh 本来就等待仿真时钟到 1.5 s 才启动 Point-LIO。出生瞬间高度和落稳后定位原点高度对应不同阶段，不能机械复制。

Point-LIO 从 IMU 初始位置建立 odom，IMU/livox_frame 在模型中的 yaw 偏角为 -1.095。故 map→odom 初始 yaw=Gazebo model yaw-1.095，当前为 -1.095，而不是 0。该计算是基于模型的初值标定，不是运行中漂移修复。

离线读取实际网格和轮组模型、YAML 加载、Launch 编译及相关包构建通过。校准数据见 modify_spawn_calibration.json，外部 Gazebo 配置变更见 modify_spawn_calibration.patch（零上下文格式）。按用户停机要求，未重新启动仿真测量最终落稳姿态；坡度、接触形变和 Point-LIO 初始误差仍可能带来小幅偏差。

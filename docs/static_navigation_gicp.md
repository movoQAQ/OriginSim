# 静态导航、指定 STL 与 GICP（2026-10-23）

按用户要求，当前仅以原有静态 PGM 生成规划地图。`dynamic_obstacles_enabled=false` 使 MapServer 不创建点云和 RadarTargets 障碍订阅；MapClient 的 require_cloud=false、require_radar=false。点云仍供 Point-LIO 和 GICP 定位使用。未来恢复动态融合需要将该开关设为 true，并重启地图节点、按需要恢复来源健康要求。

PGM 和 PCD 未修改，仍为 bringup/map/rmuc_2026.yaml、rmuc_2026.pgm 和 rmuc_2026.pcd。文件哈希与逐格运行验证见 static_gicp_runtime.json。动态模式的 Core 与旧接口测试仍通过。

Gazebo 物理和可视网格都使用用户确认的 `/home/lyu/下载/rmuc_2027.stl`，不是 rmuc_2027_new.stl。源文件和工作区 meshes/rmuc_2027.stl 的 SHA256 均为 c453658e81ea58f29f21d71e380dd87f42e7d16d49c810fb4f6ffcd14ecf2c9c。保持原生比例 1；选中 rmuc_2027_world.sdf，将网格中心平移至 (14.5,8)，原出生点 (3.4,9.5) 的地板上表面平移至 z=0.2。最终场地模型 pose 为 `10.376037598 3.718093991 -1.478711000 0 0 0`。场景配置位于 hxy/gazebo，本 nav 仓库随附文字配置补丁 gazebo_selected_stl.patch；STL 源文件由用户本机提供，没有把另一份地图替换进 PGM/PCD。

Bringup 现在加载 small_gicp_localization，停用手动 modify_map_to_odom，以保证 map→odom 的唯一发布权。GICP 的 PCD 参数使用同一个 static_pcd Launch 参数；输入 /cloud_registered，要求 frame_id=odom，所以注册输出直接是 map_from_odom。原实现直接对原始雷达点云配准，输出却作为 map→odom 发布，坐标语义不一致。

同时补全目标点云协方差、PCD 加载失败处理、最少点数及收敛/有效内点检查。初始 map→odom 使用既有出生位置配置 [3.4,9.5,0.876,-1.095]。RViz 的 initialpose 用 map_from_base×base_from_odom 初始化；GICP 以 2 Hz 尝试配准、10 Hz 发布当前修正。可用 2D Pose Estimate 重新给定初始位置。

实际 Gazebo 日志已有 GICP accepted map→odom 记录。该记录表明优化器通过收敛和内点检查，不是整场绝对定位精度证明。保留的 2026 PCD/PGM 与新 2027 物理网格存在资源差异；当前仍观察到 EXECUTION_STALLED 和足迹碰撞。未把本次启动配置验证表述为已经完成稳定导航验证。

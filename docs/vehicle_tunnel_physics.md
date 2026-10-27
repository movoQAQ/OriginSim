# 仅修改车辆的穿洞物理配置

用户要求保留现有 Gazebo 场地、场地物理属性和老 PGM。新增配置
`src/rmu_gazebo_simulator/config/tunnel_vehicle_physics.json` 只在生成车辆 SDF 时应用，
不读写场地资源。未传入 `vehicle_physics_profile` 的原有车辆启动入口保留原物理参数。

| 属性 | 原车辆 | 穿洞配置 |
| --- | ---: | ---: |
| chassis 质量 | 15 kg | 15 kg |
| chassis 重心 z，相对轮轴 | 0.12 m | 0.02 m |
| chassis 主惯量 | [0.313281, 0.313281, 0.625] kg·m² | [0.7, 2.0, 2.5] kg·m² |
| base_footprint / base_link 参考链接质量 | SDF 默认各 1 kg | 各 0.001 kg |
| aft_mapped 上部总成质量 | 1 kg | 0.25 kg |
| 纵向轴距 | 0.50 m | 0.66 m |
| 横向轮距 | 0.50 m | 0.50 m |
| chassis 接触摩擦系数 | 原模型默认 | 0.15 |
| 车体局部坐标角阻尼 | 无 | [500, 1500, 2] N·m·s/rad |

轮半径 0.0758 m、轮宽 0.024 m、车身碰撞盒 0.50×0.50×0.025 m、各碰撞形状及传感器安装均保留。
轴距修改同时作用于四个轮组安装位置和 SwerveDrive 运动学参数。加长轴距后，蓝色下方洞口需要先将车身纵轴对准行进方向，避免横着经过凸出的立柱。

`SwerveDrive` 的可选 `body_angular_damping` 对车体施加被动阻尼力矩。
在车体系中 `τ=-Cω`，功率为 `-Σ Cᵢωᵢ²≤0`，平移力为零；默认系数为零。
随机姿态与角速度下的耗散性、零系数及零角速度回归测试均保留在插件包内。
这些阻尼系数针对当前 Gazebo 求解器调试，实车应重新辨识惯量、摩擦及阻尼。

完整的两个分支联合构建与自动化复测命令见配套 nav 工作分支的
[五隧道自动化回归工具](https://github.com/movoQAQ/OriginSim/blob/codex/portable-tunnel-regression/docs/tunnel_regression.md)。
下面的手动启动例子使用其中定义的 `TUNNEL_WS`：

```bash
source /opt/ros/jazzy/setup.bash
source "$TUNNEL_WS/install/setup.bash"
ros2 launch rmu_gazebo_simulator tunnel_sim.launch.py \
  gz_world_path:=$(ros2 pkg prefix rmu_gazebo_simulator)/share/rmu_gazebo_simulator/config/tunnel_red_bottom_spawn.yaml
```

该入口加载原 `rmuc_2026_world.sdf`，采用 CPU 软件渲染及 10 Hz、720×32 雷达。
`gz_world_path` 是车辆出生位姿配置，不是场地物理模型；上述文件对应最下方红洞中心起点，默认则使用包内原有的车辆出生配置。
导航应配套使用 nav 分支的 `bringup/no_smoother_tunnels.launch.py`，并提供与出生位置一致的初始标定。

历史测试结果、实际轨迹与限制见另一检出的 nav 分支
`docs/no_smoother_vehicle_tunnels.md`。场地 5 个资源文件、原 PGM 和原 PCD 的已入库哈希记录见
`docs/no_smoother_vehicle_validation.json`；原容器内的绝对路径不是复测依赖。

## 自动化回归的可选接触观察

`tunnel_sim.launch.py` 新增默认关闭的 `regression_contacts` 和默认仍指向原场地的
`world_sdf_path`。配套测试封装在独立 trial 目录生成 world 副本，仅增加 Contact 观察系统；
机器人生成时仅增加接触传感器，车辆物理 JSON、场地源文件及几何均不修改。

请通过 nav 的 `tools/run_tunnel_regression.py` 运行完整测试。单独传
`regression_contacts:=true` 只添加传感器，仍需 world 预先加载 Contact 系统，不能当作
已经完成碰撞验收。没有收到接触消息也不能解释为没有碰撞。
本次新增观察接口仅完成源码检查和无 ROS 的结构不变性测试，尚未在当前环境运行 Gazebo。

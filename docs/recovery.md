# 停滞后退恢复（2026-10-23）

参考浙江大学 Hello World 26 赛季 HWSentryNav26：
- https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/path_executor/state/state_machine.cpp
- https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/path_executor/path_executor.cpp
- https://github.com/Polyacetone/HWSentryNav26/blob/f5f941288197e14c867d711a4c4cd85bfd7a3194/nav_executor/src/nav_executor_node.cpp

借鉴 STUCK_REVERSE 独占控制、距离/超时退出、恢复期间冻结规划、结束后一次性重新规划的设计；本实现独立编写，适配本工程全向底盘与不可变 MapView，没有引入轮腿台阶状态或复制其控制器。

## 默认行为

仅在有未完成导航目标时启用。当前位于 collision_mask 内，或 EDT 距离不超过 hard_radius_m + 0.10 m，并且连续 3 s 位移小于 0.05 m、测得速度不超过 0.05 m/s，进入后退恢复。也接受现有 RouteMonitor 的 EXECUTION_STALLED 事件；收到有效平移指令却持续静止同样可触发。零命令/规划失败不再使障碍区停滞检测失效，计时跨同一目标的规划代次保留。

以 0.15 m/s 后退，最大投影距离 0.30 m，最长 3 s。全向底盘优先沿最近 5 s 内最后一次已发布平移方向的反向退出；没有记录则取实际 base_link 的后方。方向在地图系固定，逐周期转换到控制速度使用的 base_link_fake 系。新目标清空旧方向。

整段退路使用原始占据栅格的方形几何与圆形车体检测，半径为配置车体半径（当前 0.30 m，若 footprint 更大则取更大者）+ 0.01 m。密集采样附加半步半径覆盖连续扫掠；每拍重新检查剩余通道和地图边界。仅允许退出膨胀区，已与实体占据格重叠时不强行后退。后方不安全时停车并报告 REVERSE_BLOCKED。

每目标最多 3 次尝试，冷却 5 s；被拒绝的尝试也计数，避免反复尝试被堵死的退路。超时、横向偏离 > 0.10 m、反向误差 > 0.05 m、反馈间隔 > 0.25 s、地图/TF/里程计失效都会停止。取消、新目标、急停沿用单一 PathExecutor 速度发布出口。该恢复不能修正地图与物理场景不一致或定位漂移。

恢复开始和结束时撤销旧规划代次；恢复中不接纳结果、不提交新规划，结束后从新位置向原目标重新规划。正常路径失败重试机制保留。恢复状态：`/nav_executor/recovery_status`（transient-local String），包含 STUCK_REVERSE / COMPLETE / REVERSE_BLOCKED / ABORTED_TIMEOUT_OR_DEVIATION / INPUT_INVALID / ATTEMPTS_EXHAUSTED。参数位于 bringup/params/params.yaml 和 nav_executor/config/nav_executor.yaml 的 recovery 段。

## 验证

- colcon 构建 nav_executor、bringup 通过；最终 nav_executor 增量构建通过。
- ctest --test-dir build/nav_executor --output-on-failure 通过（test_recovery，13 个 gtest 用例）。覆盖 3 s 触发、无任务运动意图不触发、实际运动/位置进展复位、距离退出、超时、冷却、次数上限、失效输入、连续扫掠/边界、后退中新增障碍、关闭配置、方向保持和任务代次失效。
- 配置 YAML 解析通过，两份 recovery 参数一致。
- 按用户要求保持仿真关闭；没有声称完成 Gazebo 或实车恢复验证。

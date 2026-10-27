# 五隧道自动化回归工具

本工具把五处隧道的批量启动、目标发布、Gazebo 真值验收和三类独立断流测试放进仓库。
它复用 `bringup/no_smoother_tunnels.launch.py`、Gazebo 的 `tunnel_sim.launch.py` 和
`tunnel_vehicle_physics.json`；无需历史测试容器，也不使用容器内的固定绝对路径。

**历史记录与本次工具验证是两件事。**
2026-10-24 的 15/15 结果保存在
[原验收 JSON](no_smoother_vehicle_results.json) 和
[原约束与验证记录](no_smoother_vehicle_validation.json)，未被重写。
本次开发环境没有 ROS 2、Gazebo、colcon，不能运行真实 Gazebo 穿越或断流测试。
本次实际执行的静态检查、合成数据测试及限制见
[交付验证记录](tunnel_regression_delivery_validation.json)；
脚本可以启动和判定测试，不代表新的 15 次穿越已经通过。

## 覆盖范围

默认 `--mode all` 包括 15 次穿越和 3 次独立安全测试。每个 case 都启动新的仿真和导航，
完成后清理本 case 的进程。场地采用原 `rmuc_2026` 资源，导航地图采用原 PGM，并保留原 PCD，
导航仍使用关闭 smoother 的已验收配置，雷达仍为 10 Hz、720×32。

| case | 中心起点 XY / m | 目标 XY / m | 初始 chassis 高度 / m | 通行方向 |
| --- | --- | --- | ---: | --- |
| `upper`，蓝色上方 | `[19.175, 14.725]` | `[19.175, 10.975]` | 0.4758 | 上→下 |
| `lower`，蓝色下方 | `[9.725, 5.225]` | `[9.725, 1.225]` | 0.2758 | 上→下 |
| `red_top`，红色上方 | `[11.7, 13.975]` | `[17.2, 13.975]` | 0.2758 | 左→右 |
| `red_middle`，红色中间 | `[11.9, 2.025]` | `[17.3, 2.025]` | 0.4758 | 左→右 |
| `red_bottom`，红色最下方 | `[11.1, 1.025]` | `[18.9, 1.025]` | 0.4758 | 左→右 |

每处的三种横向初始偏移为 `0`、`+0.04`、`-0.04 m`。这些偏移同时作用于车辆出生位置和
导航初始标定；目标仍在中心线上。模型出生高度与 chassis 真值高度不是同一个量，
工具按模型原有 frame 关系生成出生配置。全部 case 在静止、平地、车身 X 轴与地图 X 轴
一致的状态开始，等待轮速/IMU/围墙标定与地图准备完成后才发布目标。

本工具没有覆盖反向通行、任意出生点、未标定初始航向、大于 4 cm 的偏移、动态障碍物、
其他雷达频率、其他场地或实车。历史 15/15 和以后本工具的通过结果均只适用于对应记录中的
配置、源代码版本、阈值与 case 集合。

## 准备两个分支和构建工作区

目标环境为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic（`gz sim` 8）。以下命令检出本次
交付的两个配套工作分支；它们分别基于 `nav` 与 `gazebo`，尚需进行 Gazebo 实测。
待配套改动合并后，可把命令中的两个分支名分别改成 `nav` 和 `gazebo`。
如果通过补丁使用，则需将 nav/gazebo 两份补丁分别应用到对应工作树。

```bash
mkdir -p "$HOME/originsim-regression"
cd "$HOME/originsim-regression"
git clone --branch codex/portable-tunnel-regression https://github.com/movoQAQ/OriginSim.git OriginSim
git -C OriginSim worktree add --detach ../OriginSim-gazebo \
  origin/codex/portable-tunnel-regression-gazebo

export TUNNEL_NAV="$PWD/OriginSim"
export TUNNEL_GAZEBO="$PWD/OriginSim-gazebo"
export TUNNEL_WS="$PWD/OriginSim-tunnel-ws"

bash "$TUNNEL_NAV/tools/build_tunnel_regression.sh" \
  --nav-root "$TUNNEL_NAV" --gazebo-root "$TUNNEL_GAZEBO" \
  --workspace "$TUNNEL_WS" --prepare-only
```

`--prepare-only` 不依赖 ROS。它验证源码目录，在独立工作区中为所需的 17 个包建立链接，
并生成 `source-manifest.json`。重复运行可复用相同链接；遇到不同来源或无关目录会报错，
不会删除已有工作区或自动替换其他开发者的文件。

选包清单明确选择 `nav` 的 `referee_msg`，不把 Gazebo 分支的同名包同时放进工作区。
`bringup/package.xml` 还列有其他启动入口需要的 LIO、T-DT 和 RViz 包，本工具没有启动它们，
因此构建使用明确的包清单，不使用会递归引入这些依赖的 `--packages-up-to bringup`。

### 安装依赖

先按 ROS 官方安装方式准备 ROS 2 Jazzy 的 apt 源和基本安装。随后在本机执行：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-pip python3-rosdep \
  python3-colcon-common-extensions \
  ros-jazzy-ros-gz ros-jazzy-nav2-common \
  ros-jazzy-ament-cmake-auto ros-jazzy-ament-cmake-gtest \
  ros-jazzy-ament-cmake-pytest ros-jazzy-sensor-msgs-py

# 仅在这台机器尚未初始化 rosdep 时执行一次：
# sudo rosdep init
rosdep update
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths "$TUNNEL_WS/src" --ignore-src \
  --rosdistro jazzy -y \
  --skip-keys 'esdf_rviz_plugin point_lio_with_grid_map adaptive_lio frontend backend modify_map_to_odom vision_interfaces'

# sdformat_tools 使用 xmacro，但其旧 package.xml 未声明这一 Python 依赖。
# 安装到本测试工作区，不写入系统 Python 环境。
python3 -m pip install --target "$TUNNEL_WS/python_deps" xmacro
export PYTHONPATH="$TUNNEL_WS/python_deps${PYTHONPATH:+:$PYTHONPATH}"
```

这里跳过的前六项仅由其他导航入口使用，`vision_interfaces` 由其他仿真功能声明；
当前隧道入口均不导入它们。跳过这些项不意味着可以省略本工具实际启动的地图、控制器、
轮速/IMU/雷达定位、机器人描述、桥接或 Gazebo 插件。

`minimum_jerk_smoother` 仍是 `nav_executor` 的链接依赖，尽管运行时 smoothing 为关闭。
如果系统没有 `OsqpEigen`，现有 CMake 会下载并校验固定提交的 OSQP v1.0.0 与
osqp-eigen v0.11.0。因此首次完整构建需要访问对应 GitHub 仓库，或预先安装已有 CMake
支持的 OsqpEigen；构建失败必须解决后再执行仿真，不可把未启动的测试记录为通过。

### 完整构建和包内回归

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$TUNNEL_WS/python_deps${PYTHONPATH:+:$PYTHONPATH}"

bash "$TUNNEL_NAV/tools/build_tunnel_regression.sh" \
  --nav-root "$TUNNEL_NAV" --gazebo-root "$TUNNEL_GAZEBO" \
  --workspace "$TUNNEL_WS" --test

source "$TUNNEL_WS/install/setup.bash"
ros2 pkg prefix bringup
ros2 pkg prefix rmu_gazebo_simulator
```

构建脚本启用 `BUILD_TESTING`，`--test` 会执行 `map_server`、`nav_executor`、
`minco_controller`、`fake_baselink`、`landmark_alignment`、`rmoss_gz_plugins` 与
`rmu_gazebo_simulator` 七个相关包的测试，并检查 `colcon test-result`。这是选定包的构建和回归，
不包含仓库所有其他定位、视觉或决策入口。

## 无 ROS 的工具检查

在仓库根目录运行：

```bash
cd "$TUNNEL_NAV"
bash -n tools/build_tunnel_regression.sh
python3 -m compileall -q tools/run_tunnel_regression.py tools/tunnel_regression
python3 -m unittest discover -s tools/tunnel_regression/tests -v
python3 -m unittest discover \
  -s "$TUNNEL_GAZEBO/src/rmu_gazebo_simulator/test" \
  -p 'test_regression_contacts.py' -v
```

这些测试使用构造的消息和轨迹验证验收器及安全判定，例如路径绕行、缺失数据、过期真值、
停止后再次漂移、异常姿态、断流恢复和碰撞。它们不运行机器人动力学，输出的测试通过数量
不得写成 Gazebo 穿越成功数量。

## 查看计划与执行

先查看将执行的 case、配置和启动参数：

```bash
cd "$TUNNEL_NAV"
python3 tools/run_tunnel_regression.py plan \
  --gazebo-root "$TUNNEL_GAZEBO" \
  --mode all --output "$TUNNEL_WS/plans/five-tunnels"
```

`plan` 不启动 ROS/Gazebo，不产出实测通过结论。`--output` 必须是不存在或空的目录；
计划目录已有文件后，不能再把它作为 `run` 的输出目录。执行时使用新的结果目录：

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$TUNNEL_WS/python_deps${PYTHONPATH:+:$PYTHONPATH}"
source "$TUNNEL_WS/install/setup.bash"
cd "$TUNNEL_NAV"

python3 tools/run_tunnel_regression.py run \
  --gazebo-root "$TUNNEL_GAZEBO" \
  --mode all --output "$TUNNEL_WS/results/five-tunnels-$(date +%Y%m%d-%H%M%S)" \
  --startup-timeout 180 --trial-timeout 240 --sim-timeout 90
```

启动和单 case 墙钟超时用于避免初始化失败、Gazebo 冻结或进程崩溃时无限等待；
运动覆盖和连续停稳按仿真时间计算。CPU 软件渲染明显慢于实时速度时，可增加墙钟超时，
但仍须保留真实消息新鲜度检查和明确的仿真时间上限。运行器使用隔离的通信环境，
只管理自己启动的进程；不要同时在该环境中启动第二套仿真或导航。
默认会选择一个 DDS domain，并为每个 case 分配独立 `GZ_PARTITION`。
如需固定 domain，可追加 `--ros-domain-id 42`，该 domain 必须专用于本测试。

可单独运行穿越或安全套件，也可选择一处隧道的三种偏移：

```bash
python3 tools/run_tunnel_regression.py run \
  --gazebo-root "$TUNNEL_GAZEBO" --mode traversal \
  --tunnel red_bottom --output "$TUNNEL_WS/results/red-bottom"

python3 tools/run_tunnel_regression.py run \
  --gazebo-root "$TUNNEL_GAZEBO" --mode safety \
  --output "$TUNNEL_WS/results/sensor-dropouts"
```

`--tunnel` 可重复传入，用于筛选穿越 case。`--mode safety` 固定使用 `red_bottom` 中心，
不接受 `--tunnel`。在 `--mode all` 下筛选隧道只减少穿越 case，仍包含三种独立安全测试。

输出采用本次运行独立的目录，主要文件如下：

| 相对于输出目录的路径 | 用途 |
| --- | --- |
| `plan.json` | 选中的全部 case、方向、阈值、适用范围 |
| `source-manifest.json` | 两个工作树的提交/状态、受保护输入哈希、预期安装资源哈希 |
| `installed-manifest.json` | 执行前核对过的实际安装资源；环境检查失败时不生成 |
| `results.json` | 本次运行状态、穿越汇总、独立断流结果及失败原因 |
| `trials/CASE/case.json`、`spawn.yaml` | 本 case 的验收定义和出生位置 |
| `trials/CASE/contact-world.sdf` | 仅增加接触观察插件的运行时 world 副本 |
| `trials/CASE/contacts-bridge.yaml` | 本 case 的独立接触桥接配置与队列设置 |
| `trials/CASE/invocation.json` | 实际启动参数、DDS domain 与 Gazebo partition |
| `trials/CASE/events.jsonl` | 逐帧样本、底盘命令、接触与故障/恢复事件 |
| `trials/CASE/metadata.json`、`result.json` | 运行时参数检查、记录是否完整、本 case 的判定结果 |
| `trials/CASE/launch.log`、`collector.log`、`ros-log/` | 启动、采集器和 ROS 日志 |
| `analysis.json` | 以后调用 `analyze` 生成的重新分析报告 |

汇总中的逐 case 数据路径是相对路径。`invocation.json` 和安装清单保留原机路径作为来源记录，
离线分析不依赖这些路径。带走整个结果目录后仍能检查原始数据。
历史验收文件始终是基线资料，不会被本次运行器覆盖。

安装核对覆盖 launch、配置、模型 XML、地图和定位 Python 的逐文件哈希。
记录同时包含关键 C++ 源码及已安装二进制的指纹，便于追查混用旧构建的问题；这些指纹
本身不能证明二进制由当前源码重新编译，因此仍需执行上述完整构建并保留 colcon 日志。

若 ROS、Gazebo 或已 source 的安装资源不满足要求，`run` 返回 2，生成
`execution_status: "blocked"` 的 `results.json`，完成 case 数为 0；这不算一次穿越失败，
也不算已经运行过机器人。启动后的 case 超时、观测缺失或物理验收失败则保留为失败结果。

## 验收依据

穿越判定要求同时具有完整的传感器、任务状态和碰撞证据，至少检查：

- Gazebo 真值在限定的横向通道范围内覆盖洞内至少 95% 的 0.1 m 分段，并按指定方向经过
  洞口两侧至少 0.3 m。绕行到终点不算穿越；丢帧之间不凭空补一条连续轨迹。
- 最终真值 XY 到目标误差小于 0.30 m，且本次目标的执行器报告 `SUCCEEDED`。
- 底盘命令与真值线速度小于 0.05 m/s，角速度小于 0.05 rad/s，连续保持至少 3 s 仿真时间。
  真值或命令过期、时钟停滞、停止后重新移动都会阻止虚假的“已停稳”结论。
- 实际加载的 smoother 参数为 `false`，地图及受保护资源通过哈希核对。
- 最大车体倾角必须小于 30°。这一拒绝阈值是本次新增要求，不是历史验收得出的安全极限。
- 记录车体倾角、洞内横向偏差、定位偏差、里程计中断、执行停止及其恢复。
  执行器日志中的超时短停不会被误写成全程无中断。
- 碰撞观察链路必须存活。缺少碰撞消息属于证据缺失，不是“零碰撞”。

新工具的洞内范围来自原 PGM 上沿中心线连续、平行窄壁的栅格范围，显式保存在
[`cases.json`](../tools/tunnel_regression/cases.json)，没有修改 PGM。
对应的中心见证带半宽等于该段栅格净宽的一半减去既有 0.30 m 机器人半径：

| 通道 | 洞内起点 XY / m | 洞内终点 XY / m | 中心见证带半宽 / m |
| --- | --- | --- | ---: |
| `upper` | `[19.175, 14.0]` | `[19.175, 11.6]` | 0.125 |
| `lower` | `[9.725, 4.4]` | `[9.725, 1.85]` | 0.125 |
| `red_top` | `[12.9, 13.975]` | `[15.85, 13.975]` | 0.075 |
| `red_middle` | `[13.15, 2.025]` | `[16.0, 2.025]` | 0.075 |
| `red_bottom` | `[12.15, 1.025]` | `[18.35, 1.025]` | 0.025 |

历史容器脚本没有随仓库保存，以上范围是此次重新从 PGM 推导的，不能宣称与历史脚本的
内部范围完全相同。本工具的逐帧记录、case 定义与阈值共同定义新的验收证据。
历史的最大误差 0.1413 m、最大洞内偏差 0.0198 m、最大倾角 5.62° 是历史观测值，
不是自动替代本次验收阈值的普遍安全上限。

### 碰撞观察

Gazebo 的测试观察接口仅在 `regression_contacts=true` 时启用。工具复制原场地 SDF 到当前
case 目录，只在副本中增加 Contact 观察插件；插件在机器人生成前加载。运行时生成的机器人
增加接触传感器，并通过单独的 Contacts bridge 输出观察结果。生成副本与原 world 在移除
观察插件后应具有相同的 XML 树；测试覆盖这一不变性。

场地几何、碰撞形状以及车辆质量、惯量、摩擦等物理配置均保留。生产入口默认仍加载原 world，
不启用接触观察；原始 PGM、PCD、STL、场地 SDF 和已验收的车辆物理 JSON 均不改写。

判定区分轮胎的支撑接触和禁止接触：轮胎与地面/坡面的支撑接触用于证明观察链路确实收到
接触信息。支撑接触要求是已知轮胎与环境之间的接触，且单位法向的 `abs(z) >= 0.5`；
车身等非轮胎部件的接触、轮胎与壁面的接触会记录为失败。接触法向及其分类规则
随原始数据一起保留，避免把正常落地误计为“撞墙”，也避免把无消息误计为“无碰撞”。
有效支撑接触证据的最大年龄为 2 s 仿真时间和 5 s 墙钟时间；Contact 系统只发布非空接触，
因此没有单独伪造“空接触心跳”来通过新鲜度检查。超过时限没有支撑证据会使验收失败。
这套显式接触准则也是新增要求，可能拒绝旧测试没有单独分类的台阶接触，不能据历史 15/15
推断它必然通过。新增观察还可能带来性能开销，首次 Gazebo 实测须核对消息速率和 CPU 负载。

### 独立雷达、IMU、轮速断流

三种安全 case 都从 `red_bottom` 中心的全新仿真开始。工具先等待标定完成，并确认车辆
在执行原目标且确实运动，再只阻断一个传感器向导航消费者转发的数据：

| case | 停止转发的输入 | 必须继续的证据 |
| --- | --- | --- |
| `lidar` | 原始雷达点云 | 原始传感器、IMU、轮速、Gazebo 真值、仿真时钟 |
| `imu` | IMU | 原始传感器、雷达、轮速、Gazebo 真值、仿真时钟 |
| `wheels` | 轮速/转向 `JointState` | 原始传感器、雷达、IMU、Gazebo 真值、仿真时钟 |

这与历史“同时冻结雷达/IMU/轮速桥接”的测试不同。工具验证选中的转发输入确实停更，
其他输入和独立真值/时钟继续推进；同时检查里程计抑制、零底盘命令和实际停车。
故障保持期间不靠测试器持续发布零速度来替导航系统通过停车验收。
解除阻断后还要求输入恢复、里程计恢复，以及在不重新发布目标的情况下继续完成原任务。
缺少任一证据、观察超时或未自动恢复都应返回失败并保留日志。

当前断流判定使用独立的测试时限，不修改生产节点的超时参数：雷达断流后的零命令截止为
1.3 s 仿真时间，IMU 和轮速为 0.9 s；墙钟已过 2 s 时也开始要求零命令。
断流保持至少 3 s 仿真时间和 1 s 墙钟时间；2 s 仿真时间后要求实际线/角速度低于
0.05 m/s、0.05 rad/s，并持续停稳，停稳观察窗口的 XY 漂移不超过 0.05 m。
恢复输入后须在 15 s 仿真时间、60 s 墙钟时间内
观测到新输入、新里程计、实际运动和至少 0.1 m 的恢复位移，随后完成原目标的全部穿越检查。
这些是本次新增工具的验收要求，不是本次环境已经测得的安全停车时间。

还会单独检查 `/odom` 是否停止输出；只得到零底盘命令不足以通过。允许的最后输出时间
相对于故障开始如下，包含已有数据新鲜度门限之外的调度余量：

| 断流输入 | 最后里程计时间戳 / 仿真秒 | 最后接收时间 / 墙钟秒 |
| --- | ---: | ---: |
| 雷达 | 0.7 | 1.0 |
| IMU | 0.3 | 0.5 |
| 轮速 | 0.3 | 0.5 |

逐次结果保留 `last_odom_stamp_after_fault_s`、`last_odom_receipt_after_fault_wall_s` 和
`suppression_observed`。持续发布旧时间戳的里程计也会被接收时间检查识别。真实恢复位移
从 XY 真值重新计算，不能以车体落地时的高度变化代替导航恢复。

## 结果审查

运行器的退出状态与汇总共同用于自动化验收。选择子集时，只能报告该子集，不能写成
“五隧道 15/15”。中途退出、启动失败和超时不能从分母中删除；单个 case 失败应保留失败原因。
任何报告都应同时给出分支提交、脏工作树状态、资源哈希、运行参数和测试时间。

| 命令/退出码 | 含义 |
| --- | --- |
| `plan` 返回 0 | 计划成功；没有运行 Gazebo |
| `run` 返回 0 | 所选全部 case 均完成并通过 |
| `run` 返回 1 | 至少一个执行或验收失败 |
| 返回 2 | 输入、源文件约束或运行环境错误；查看错误信息及已生成的报告 |
| `run` 返回 130 | 用户中断；已完成结果和当前 case 日志保留 |
| `analyze` 返回 0/1 | 已保存证据的重新分析通过/失败；新增实测次数仍为 0 |

可在无 ROS 环境重新分析已保存的记录：

```bash
python3 tools/run_tunnel_regression.py analyze "$TUNNEL_WS/results/red-bottom"
```

重新分析只对已有记录运行判定，单独写入 `analysis.json`，保留原 `results.json`。
它不启动 Gazebo，也不新增实测次数；合成记录仍标为 `synthetic`，缺失记录不会计为
已完成的 Gazebo case。已有逐帧文件的哈希与原结果不一致时会判失败。
正式提交测试结论时，应附上完整结果目录及其日志；如果日志或逐帧记录缺失，
只保留汇总中的数值不足以证明这次回归可重复或已完成。

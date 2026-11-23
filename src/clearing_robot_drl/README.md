# clearing_robot_drl

单层 SAC-HP：3 个 Actor 的对角高斯以一、二阶矩匹配，再与固定方差 0.3 的 MPPI 先验高斯相乘。MPPI 输入是 [-1,1] 的动作，先以 `atanh` 变换到潜空间；动作采样经 `tanh` 并使用雅可比修正计算对数密度。双 Q、目标网络和自动熵温度用于 SAC 更新。观测 28 维，前 20 维为基础项，后 8 维为扩展项；`base` 消融将后 8 维置零。两维动作顺序为线速度、角速度，[-1,1] 分别对应 [0.50,1.00] m/s、[-0.12,0.12] rad/s；服务端转换巡航动作；停车独立使用零速指令，起步和制动保留速度过渡。速度归一化已更新，应使用新实验重新训练。

构建前需要 ROS 2、PyTorch、NumPy、Gazebo、MultiMapNav 及 `clearing_robot_plugins`；导出需要 Python `onnx`。加载 ROS 2 和两个工作空间后，分别在三个终端运行以下命令。周期评估由训练进程按需启动独立的无界面 evaluation 环境，仿真按自然时间运行。

```bash
# 终端 1
ros2 launch clearing_robot_simulation clearing_robot.launch.py
# 终端 2
ros2 launch clearing_robot_plugins navigation.launch.py map_mode:=gridmap controller:=training
# 终端 3，等导航模式就绪后执行
/data/projects/miniconda3/envs/multimapnav_drl/bin/python -m clearing_robot_drl.runtime \
  --mode train --policy hybrid --scene training --run-id rolling_01
```

### 手动评估、恢复和导出

单独评估时，在所选场景启动仿真与训练控制接口，再执行对应策略的评估。批量对比时给不同策略使用相同种子和回合数。

```bash
drl_python=/data/projects/miniconda3/envs/multimapnav_drl/bin/python
"$drl_python" -m clearing_robot_drl.runtime --mode eval --scene evaluation \
  --policy hybrid --weights /path/trial1-step-weights.pt --seed 42 --run-id hybrid42
"$drl_python" -m clearing_robot_drl.runtime --mode eval --scene evaluation \
  --policy mppi --seed 42 --run-id mppi42
"$drl_python" -m clearing_robot_drl.runtime --mode train --policy hybrid \
  --resume /path/trial1-step-checkpoint.pt --run-id trial1
"$drl_python" -m clearing_robot_drl.export /path/trial1-step-weights.pt /path/policy.onnx
```

`--policy no_prior` 是去先验消融，`--policy base` 去掉扩展观测。两种消融需分别训练后使用对应权重评估。评估采用确定性融合均值，MPPI 基线直接使用先验，均不执行优化。固定 ONNX opset 17，输入 `observation` float32 [1,28]、`prior` float32 [1,2]（归一化 [-1,1]），输出 `action` float32 [1,2]；输出是融合潜空间均值的 tanh。

回合开始先 STOP，Gazebo 复位到固定位置，随后 `/clearing/reset_estimator` 重置 FAST-LIO。每回合以 `seed + episode` 产生候选目标，通过 `/compute_path_to_pose` 检验路径、终点和可达性。碰撞、过大横滚/俯仰、目标到达或超时结束回合；错误则单独记录且不计成功率。目标范围、地形高度、初始位置、限幅、时间、收敛门限、场景名均可通过 `--help` 修改；需根据所选地形配置可通行的目标范围和高度。

`records/<run-id>/` 有运行配置、逐步 `steps.csv`、回合 `episodes.csv`、错误 `errors.jsonl` 和最近窗口 `metrics.json`。奖励为 `2*(上次目标距离-本次目标距离) - 0.02*dt - 0.04*dt*横向误差²`，成功 +8，碰撞或倾覆 -20，超时 -10。误差是实际位置到完整路径最近线段的距离及对应航向差；时间使用仿真秒。收敛判据基于周期性独立评估，见下文。评估时除报告成功率外还应比较独立场景、相同种子和目标的逐回合结果；若不同场景规划可达性不同，需核对实际目标坐标而非假设完全配对。

`models/` 保存仅推理的 `*-weights.pt` 和包含 replay、优化器及 Python/NumPy/PyTorch RNG 的 `*-checkpoint.pt`；每个快照名包含 run、累计步数、回合和唯一标识，创建时拒绝覆盖；`--records`、`--models` 可覆盖。仅加载可信来源的 PyTorch 文件。服务或规划错误写入错误日志，不构造假动作样本；复位失败时不会继续该回合。

观测约定：0–1 为 2.5 m 前视点的前向、侧向坐标除以 12 m；2–3 为航向误差 sin/cos；4–5 为 LIO 测速 v/1.0、ω/0.12；6–7 为实际指令按相同尺度归一化；8–15 为以正前方开始、逆时针每 45° 的距离除以 12 m；16–18 为局部路径末点前向、侧向和距离除以 20 m；19 为路径有效位；20–21 为实际指令与 LIO 速度之差按相同尺度归一化；22 为 LIO 侧向速度/1.0；23–24 为 roll/pitch 除以 0.3 rad；25 为中心坡度/0.3 rad；26 为前方 1 m 高差/0.25 m；27 为实际指令反馈有效位。服务端对归一化结果限幅。

默认输出路径指向本项目源码包的 `records/` 和 `models/`；迁移目录时可设置 `CLEARING_DRL_SOURCE`，或使用 `--records`、`--models` 指定目录。此机器现有环境 `/data/projects/miniconda3/envs/multimapnav_drl` 提供 PyTorch、ONNX 和 ONNX Runtime；执行 ROS 节点时还需正确 source ROS 及工程环境。

### 周期独立评估与收敛停止

训练默认每累计 5000 步，在当前回合结束并 STOP 后保存权重快照，按需启动独立的无界面评估环境与评估进程（20 回合、种子 100000）。评估环境自动选择回环网络端口，使用单独的 Fast DDS 发现服务和 Gazebo master，不需预先启动评估场景。`--eval-scene` 选择评估场景；训练域保持原有配置。以本次启动日志中的机器人和导航就绪标记为准，`--eval-startup-timeout` 默认 300 秒，`--eval-wall-timeout` 默认 7200 秒，评估结束、失败或中断后自动回收本次创建的进程。各阶段日志分别以 `-discovery.log`、`-bringup.log` 和 `-worker.log` 命名。可通过 `--eval-every-steps`、`--eval-episodes` 和 `--eval-seed` 调整评估频率和规模；错误或不完整的评估不会通过收敛门限。

默认用最近 3 次完整独立评估及更早一次熵基线判断以下四组门限，全部通过才自动停止并保存：

1. 回报均值的极差除以窗口回报均值绝对值小于 5%（分母下限 1e-6）。
2. 成功率极差小于 3 个百分点，且每次成功率不低于 90%。
3. 采样熵相对早期评估至少下降 0.05，最近窗口极差和相邻跳变量均不超过 0.1。熵为融合策略采样的 `-log π(a|s)`，包含 tanh 雅可比，评估所执行动作仍是确定性均值。
4. 每次碰撞率不超过 5%，且其窗口极差不超过 3 个百分点；偏航角速度反向幅度均值不超过 0.04 rad/s、极差不超过 0.02 rad/s；相邻实际线速度变化均值不超过 0.1 m/s、极差不超过 0.03 m/s。低于死区的实测角速度不计符号，变化率均按回合等权平均。

各数值可通过 `--convergence-window`、`--reward-relative`、`--gate-success`、`--success-spread`、`--entropy-drop`、`--entropy-spread`、`--entropy-jump`、`--gate-collision`、`--collision-spread`、`--max-oscillation`、`--oscillation-spread`、`--max-speed-change`、`--speed-change-spread` 和 `--yaw-deadband` 调整。仿真时间和实际过程变化可能导致不同场景的可行目标不同，须核对各回合目标坐标。`metrics.json` 还提供超时率及回报分项。达到最大回合、收敛、运行错误或 Ctrl-C 时，`stop.json` 和检查点记录停止原因；Ctrl-C 时当前未完成回合不计入评估。训练端等待评估时物理仍自然运行，车辆维持 STOP。

`--max-steps` 以跨回合累计训练步数为上限（默认 1000000）；`--checkpoint-every-steps` 控制中途快照间隔（默认 5000），`--record-every-steps` 控制逐步 CSV 的批量刷新间隔（默认 50）。回合结束也会刷新回合表；进程异常退出可能丢失尚未刷新的逐步日志。中途快照先 STOP 并在保存后刷新观测。恢复未完成回合的检查点时，以相同回合号从初始位置重新开始，已保存的 replay 和累计步数保留；因此同一回合号可能有两段逐步记录。若因最大步数停止后希望继续训练，恢复时须提高 `--max-steps`。`maximum_steps` 不计入评估；`planning_timeout` 计为导航失败和超时，但无动作样本不计入策略熵、奖励或平滑性均值。每次评估的四组门限及其证据随该次记录保存在 `evaluations.json`，记录包含训练累计步数、权重、检查点、场景和种子。源码位置按当前源码和已安装工作空间的路径自动发现，非标准安装可设置 `CLEARING_DRL_SOURCE` 或覆盖输出目录。

默认从对应场景的三维目标清单采样，再由全局规划器检查可达性。`--planning-timeout` 设置单次规划墙钟超时，超时后请求并确认取消，再重试新候选；达到 `--planning-timeout-retries` 后记作 `planning_timeout` 回合。若取消无法确认则记作运行错误并终止，避免遗留规划任务。

示例（须先 source ROS 与工作空间环境）：

```bash
# 第一终端：启动训练场景和训练控制服务
ros2 launch clearing_robot_plugins bringup.launch.py scene:=training map_mode:=gridmap controller:=training
# 第二终端：启动训练；周期评估由训练命令自动创建独立环境
ros2 run clearing_robot_drl clearing_drl --mode train --scene training --run-id trial1
```

训练默认从训练场景目标清单采样，经真实全局路径验证后接受。局部路径暂时封堵时停车等待，`--local-planning-timeout` 默认 3 个仿真秒。振荡幅度取实测角速度越过死区并反向时的半峰峰差（rad/s），非反向步记 0；同时保存反向频率。速度平滑性为相邻实测线速度的绝对差（m/s）。在评估中策略用确定性均值动作，指标中的熵由额外抽样的 `-log π(a|s)` 得到，使用 tanh 变换的雅可比，报告归一化动作域中的微分熵（nat）。

奖励里程项每前进 1 m 给 2 分，折返等量扣分；时间项每秒扣 0.02，保持连续前进的压力；横向误差平方按每秒 0.04 加权，远离全局参考时成本快速增长。终端碰撞或倾覆扣 20，与约 10 m 净进度相抵，成功加 8，超时扣 10。该尺度用于初始实验配置。论文第 4 节的策略集成、固定方差行为先验和高斯乘积用于本实现；本项目用 MPPI 替换 DWA，动作改为底盘线速度和角速度，使用连续仿真时间和全局路径跟踪奖励。各个 Actor 通过融合策略更新，Critic 采用共享双 Q。

奖励趋势上限由 `--reward-trend` 控制（相对窗口均值，默认每周期 1.5%）。质量趋势用各指标波动上限乘 `--quality-trend-fraction`（默认 0.45）约束持续变化；熵还需低于 `--entropy-upper-bound`（默认 0.8 nat）。

定位、滚动地图或驱动反馈短时失新时，环境先停车并等待新观测；超过 `--observation-timeout`（默认 12 秒墙钟）记为规划超时并触发复位。

### 场景目标和自定义地图

默认 `--goal-source scene` 使用 `ament_index_python` 定位 `clearing_robot_simulation`，分别读取已安装的 `worlds/training_goals.json` 和 `worlds/evaluation_goals.json`。目标包含实际地形 x、y、z，候选点仍由 `/compute_path_to_pose` 检查路径和终点；文件缺失或场景不匹配时明确报错。每回合由 `seed + episode` 决定目标顺序，同一轮遍历不重复。终点航向由当前位置指向目标；到达判定仍采用水平距离，z 用于全局规划。

格式示例：`{"scene":"training","frame_id":"map","goals":[{"x":30.0,"y":8.0,"z":0.5}]}`。数值仅用于说明格式，实际 z 须与场景地形一致。支持 `frame: world`，前提是 world 与 map 原点和轴一致。自定义 PCD 或其他地图可指定 `--goal-source /path/my_goals.json`，也可指定 `--goal-source rectangle --goal-x-min 10 --goal-x-max 20 --goal-y-min -5 --goal-y-max 5 --goal-z 0`；单独指定任一矩形边界或 `--goal-z` 也会选用矩形模式，其余边界采用默认值。矩形 z 固定，需根据地图地形设置。

周期评估默认使用 `--eval-scene` 对应清单，不继承训练的目标来源；通过 `--eval-goal-source /path/eval_goals.json` 或 `--eval-goal-source rectangle` 显式指定评估目标，后者使用训练命令中指定的矩形参数。每套仿真须已安装与场景匹配的目标清单。默认单回合仿真时间上限为 240 s，可通过 `--timeout 260` 调整；最高线速 1.00 m/s 时直线行驶 50 m 理论上需约 50 s，避障、转弯和低速会延长时间。

训练终端用 `--log-every-steps` 调整进度日志间隔，默认每 20 步输出一次。

### 新版随机训练地形与原目录续训

从兼容的同一运行检查点恢复时可用 `--resume latest --run-id <原运行ID>`。
当前原运行 `56ba433162ac` 已有步数 4660 的完整检查点。请使用对应的仿真和导航场景参数。

```bash
ros2 launch clearing_robot_simulation clearing_robot.launch.py world_file_name:=training_rough_v2
ros2 launch clearing_robot_plugins navigation.launch.py scene:=training_rough_v2 map_mode:=gridmap controller:=training
/data/projects/miniconda3/envs/multimapnav_drl/bin/python -m clearing_robot_drl.runtime --mode train \
  --policy hybrid --scene training_rough_v2 --resume latest --run-id 56ba433162ac --timeout 360
```

三条命令在三个已加载同一工作区的终端运行。连续起伏的新版训练地图由同一地形网格生成世界碰撞面、PCD 和采样数据。
每次复位随机选择带朝向的起点，筛查车体包络、最小转弯空间、坡度、台阶和障碍间距，确认定位后再以全图候选目标请求 A*。
新的起终点写入 `sampling.jsonl`。首次采集新场景交互时，`scene_changes.jsonl` 记录启用步数、来源检查点、地形摘要及回放处理。
续训在原目录追加 CSV，原始 `run.json` 保留，追加的会话设置写入 `sessions.jsonl`。
切换场景时恢复网络、优化器、温度、随机数及全局步数，经验池重新积累；旧经验留在原检查点。
独立评估仍使用 `evaluation`，其新的收敛窗口从场景迁移后重新计算。若步日志比检查点更晚，启动时会拒绝追加以避免重复。

### 停稳、观测版本与产物布局

仿真复位前反复请求 STOP，要求停车后的新鲜执行反馈和 Gazebo 实测三轴线速度、角速度同时达标，并持续至少 0.35 个仿真秒。仿真秒和墙钟各有限时。执行反馈是轮速指令反算，不能代替 Gazebo 实测运动。复位检查结果及时间、速度保存在 `resets.jsonl`。

LocalMap 的 GridMap 负责地面和地形，OctoMap 负责障碍。输入维度、顺序及动作尺度保留；障碍标记和净空相关观测值会变化。新检查点记录观测版本，切换语义时恢复网络、优化器、随机数和累计步数，并清空运行中的回放及收敛窗口，旧回放仍在原检查点。

新运行目录采用 `实验-场景-运行ID`，包含 `parameters/`、`weights/`、`checkpoints/`、`records/`。快照名包含实验、场景、训练步、回合及用途。旧运行的 CSV 和初始配置继续沿用原路径，后续快照存入原运行目录下的新子目录。续训可以用 `--resume /绝对路径/原运行目录` 或 `--resume /绝对路径/...-checkpoint.pt`，也可以使用 `--resume latest --run-id <原运行ID>`。支持 `--check-resume` 只读预检。仅加载 weights 的命令适用于权重初始化及评估。

### 复位恢复与 TensorBoard

复位后先确认当前进程重启后的 FAST-LIO 原始里程计及同帧桥接输出，再等地图匹配、更新的动态 TF、局部地图和速度反馈。新观测与本次移位时间比较，确认后的位姿符合起点约束才请求全局路径。阶段状态、耗时及失败原因写入 `records/recovery.jsonl`，实测停稳写入 `records/resets.jsonl`。

`--recovery-attempts` 默认 3 次，包含首次尝试。`--reset-ready-timeout` 默认每次 20 个墙钟秒，`--planner-recovery-timeout` 默认 30 个墙钟秒。恢复用尽后会停车、保存 `recovery_wait` 检查点，并在终端定期提示；按 Ctrl+C 安全结束后可从同一检查点恢复。独立评估会记录错误并返回主流程。正常学习期间，`--progress-timeout 20` 和 `--progress-min-distance 0.5` 分别设定可无有效路径进展的仿真秒数和最小路径弧长增量（米）。触发时以规划超时结束当前回合，诊断写入 `records/control_events.jsonl`。

每个运行目录的 `tensorboard/` 保存事件文件，以累计环境步数为横轴。可从记录根目录同时查看多个运行。终端命令可使用训练环境的 `python -m tensorboard.main --logdir /路径/clearing_robot_drl/records --port 6006`。

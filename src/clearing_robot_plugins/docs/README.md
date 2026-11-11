# 大型清障机器人 MultiMapNav 接入

本包保存清障机器人专用插件、配置、启动文件、先验地图和三维碰撞包络。运行 MultiMapNav 原有服务器、自动地图选择器和生命周期管理器，加载清障机器人自己的三模式 profile。

## 构建和启动

```bash
cd /data/projects/ros2_ws/MultiMapNav_Simulation
source /opt/ros/humble/setup.bash
bash src/multimap_nav_simulation/scripts/build_livox_workspace.sh
source install/setup.bash
ros2 launch clearing_robot_plugins navigation.launch.py
```

构建脚本叠加相邻 `MultiMapNav/install/local_setup.bash` 中的接口和服务器。默认启动机器人、FAST-LIO2、五路雷达融合、自动地图选择器、完整导航服务器及 RViz。无界面测试使用 `headless:=true use_rviz:=false`。

不要同时启动两份机器人或导航栈。已有机器人时使用 `start_simulation:=false`，已有 FAST-LIO2 输入应配置为 `primary_lidar_topic:=/clearing/lidar_fused primary_lidar_frame:=imu_link fused_lidar_in_imu:=true`。
初始位姿参数为地图中部署位置的先验种子 `[x,y,z,qx,qy,qz,qw]`，默认出生在原点；RViz 的初始位姿工具也可重新设置。

底盘启用前，必须从 `/joint_states` 确认七个关节处于收臂角、速度接近零且持续稳定。即使机械臂 Action 的 DDS 回复丢失，也只在这一实际反馈确认通过后启用底盘。

## 仿真和在线建图

使用独立建图入口，启动大型清障机器人、五雷达融合、FAST-LIO2和建图RViz：

```bash
ros2 launch clearing_robot_plugins mapping.launch.py
# 切换至广场场景，并设置适合大型车体的出生位置
ros2 launch clearing_robot_plugins mapping.launch.py map_name:=flat_3 spawn_x:=-6.0 spawn_y:=-7.0
# 自定义场景和PCD输出位置
ros2 launch clearing_robot_plugins mapping.launch.py \
  world_file:=/absolute/path/scene.world \
  map_output_file:=/absolute/path/my_map.pcd
```

`map_name` 默认 `clearing_enclosed`；还可使用 `flat_1..5`、`terrain_1..5`、`floor_1..7` 或 worlds/ 下的相对名称。`world_file` 优先于 `map_name`。这些名称选择的是Gazebo场景，建图算法仍为FAST-LIO2。地图目录原有出生点属于小型机器人，本入口不套用它们；根据大型车体尺寸设置 `spawn_x`、`spawn_y`、`spawn_z`、`spawn_yaw`。目录中的场景会按平面边界检查履带支撑区域，明显越界时在启动前报错；这不代替三维碰撞或坡度检查。部分窄通道、低层高场景无法容纳此机器人。

在另一个已加载工作区环境的终端中低速驾驶：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.15
```

建图过程中保存地图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

默认PCD输出到启动时当前目录的 `maps/<场景名_时间戳>/map.pcd`，启动日志打印完整路径；也可用 `map_output_file` 指定。地图是 `/Laser_map` 的累计点云，当前实现需在退出前调用保存服务，不能依赖退出自动保存。RViz同时显示累计地图、配准扫描、运动路径及收臂包络。

建图模式不加载先验PCD、支撑层、导航定位或规划服务器。五路CustomMsg在自体过滤后组合到IMU坐标，FAST-LIO2在收臂确认后启动。TF为 `map -> odom -> base_link`，其中 `map -> odom` 为单位变换，原点采用FAST-LIO2初始化时的IMU参考系，与Gazebo世界坐标不必重合。此入口没有先验匹配或回环优化，因此长期建图仍可能累计漂移。`headless:=true use_rviz:=false` 可用于无界面运行。

建图测试记录见 [建图验证](mapping_validation.md)。

## 自动与强制地图模式

默认 `forced_mode:=auto`，由 MultiMapNav 的 `navigation_mode_selector_server` 分析原始 PCD；`lifecycle_manager` 获取结果后，在各服务器 configure 前应用清障机器人 profile。当前封闭场景包含地面、墙、箱体及顶棚，原选择器测得多高度结构比例约0.817，自动选择 `octomap`。体型不是唯一的选图依据；自动选择规则仍能对平地选择 Costmap2D、连续起伏选择 GridMap。

```bash
# 自动选择（默认）
ros2 launch clearing_robot_plugins navigation.launch.py forced_mode:=auto
# 分别强制三种模式；只运行其中一个启动命令
ros2 launch clearing_robot_plugins navigation.launch.py forced_mode:=octomap
ros2 launch clearing_robot_plugins navigation.launch.py forced_mode:=gridmap
ros2 launch clearing_robot_plugins navigation.launch.py forced_mode:=costmap2d
```

| 模式 | 实际地图插件及输入适配 | 规划器订阅 |
| --- | --- | --- |
| OctoMap | 框架 `OctoMapPlugin`，保留三维占据叶节点及实际尺寸 | `/octomap_map` |
| GridMap | 清障 `GridMapAdapter` 调用框架 `GridMapPlugin`，过滤支撑面以下的地板底面 | `/grid_map` |
| Costmap2D | 清障 `Costmap2DAdapter` 调用框架 `Costmap2DMapPlugin`，按4.0×0.4 m履带接触窗口拟合地面，排除地下点与远高于车体的顶棚 | `/costmap` |

每个模式都使用论文全局规划、Hybrid A*、MPPI、FAST-LIO2和清障后退/旋转恢复，不加载 tracked_robot 的算法参数或真值里程计。三种模式都有独立参数命名空间，限速、底盘及机械臂包络从同一机器人配置复制。

OctoMap 模式按占据体素与18个有向盒体的体积相交检测碰撞，不能只检查体素中心。GridMap/Costmap2D 的占据、未知单元作为保守的垂直列与整车投影相交；原始三维先验和实时整车安全检查在所有模式下均保留。因此强制二维模式可能拒绝能从高度上绕开的通道，强制模式不会关闭碰撞检测。

三维地图表示不改变履带底盘的地面支撑约束：论文全局搜索仍运行在给定支撑层上，OctoMap用于保留空间障碍高度，不等于新增跨楼层的连通规划。

查看实际选择结果和地图使用情况：

```bash
ros2 service call /navigation_mode_selector_server/get_selected_navigation_mode multimap_nav_msgs/srv/GetSelectedNavigationMode '{}'
ros2 topic echo /clearing/map_backend
```

`map_backend` 分别报告全局、局部、轨迹、恢复服务器实际采用的模式、地图话题和占据单元数。地图未到达时保持停车。模式在启动时选择，切换时应结束当前启动进程后使用相应参数重新启动。

`config/modes.yaml` 使用框架原有选择器参数和 profile 接口。清障机器人尚无独立CPU耗时标定，资源惩罚暂设为0，不沿用 tracked_robot 的CPU标定数据；地图结构与表示能力约束仍参与自动选择。

## 算法与碰撞检测

- 全局：保留论文的双向指数加权 A*、三维坡度距离、节点过滤、爬坡/侧倾限制、三次 Bézier。增加可前进/倒车的解析连接与SE(2)运动学修复，修复栅格拐角对大型非圆车体不可执行的问题。实现细节见 `paper_traceability.md`。
- 局部：SE(2) Hybrid A* 运动原语搜索，优先保留可通行的全局参考段；阻挡时搜索含转向/倒车的局部路径。
- 轨迹：MPPI 采样预测和软最小权重更新，限制速度、加速度、加加速度；加权结果再次检测，另验证计入加加速度、位姿年龄和反应时间的停车包络。
- 恢复：低速后退 0.6 m、小角度旋转约 15°；每个周期都检查完整三维扫掠体。

`config/robot_model.yaml` 的 18 个有向盒体由收臂姿态的 URDF 自动生成，包含底盘、履带、回转平台、机械臂、铲斗和雷达。盒体几何既用于 `/clearing/collision_model` MarkerArray，也用于全局平滑复核、局部规划、MPPI 预测及恢复检查。机器人沿路径运动和转动的中间状态也要通过检测，而不仅是离散端点。

铲斗包络固定为底盘坐标轴方向，平地收臂时与地面平行，名义尺寸约 1.528 × 2.550 × 0.630 m。小臂 link4 分为四个沿长度切分的紧凑盒体，合计名义体积比原单盒减少约39%；切分包括原始几何边与切平面的交点，不会漏掉跨切面部分。所有原始体积仍被保守包围，几何拟合额外留出0.025 m。底盘、履带及雷达坐标轴保持URDF定义，避免拟合轴排列改变传感器外参。

当前收臂名义包络约为 9.34 × 3.28 × 4.56 m，另加 0.20 m 碰撞余量。碰撞 Marker 的余量直接读取同一 `scene.collision_margin` 配置。启动时校验模型、机械臂几何、雷达宏、收臂角、履带尺寸及默认地图来源哈希；改变几何后必须重新生成资产。

模型针对配置中的收臂姿态。关节状态偏离该姿态、传感器过期或定位失去质量时，控制停止。前方两个 Avia 小角度外偏 ±10°以增加中线重叠，原来的 ±45°在外沿安装位置会产生明显前向盲区。

`/clearing/obstacles` 是所有运动插件共享的地图坐标障碍观测。新增障碍至少保留 10 秒；超过时限后，仅删除距离底盘超过 13 m 的旧体素。附近障碍在转弯遮挡或定向雷达盲区内继续保留，不能因为没有新回波就把它当作空地。当前没有射线自由空间清除，移动目标离开后，附近的历史占据可能持续保留并造成保守停车；离开该区域后才清除过期记录。已知静态表面由持久先验负责。传感器或定位本身不再更新时，仍须停车，障碍记忆不能代替数据健康检查。

局部搜索使用车体朝向、挡位与转向状态，底盘障碍距离场仅提供启发值。解析连接包括OMPL的Reeds–Shepp/Dubins、带符号切线的三次曲线及平行侧移连接；所有连接都复核完整三维车体和曲率约束。正常导航不生成原地转向点，纯朝向目标通过有移动距离的机动完成；无法容纳机动时报告不可行。它不保证最短路径。已验证的局部剩余路径在复用前重新检查整车扫掠体，路线不变时保持路径时间戳，避免每次局部更新都重启轨迹跟随器。

车体回波必须在采集时刻的刚体坐标系内剔除，再做运动对齐，避免转弯时把旧铲斗回波投到当前模型外。新增障碍在进入记忆前须在 0.45 m 邻域内获得至少 3 个体素点支持，以去除孤立射线异常；这也意味着非常稀疏的小目标可能暂时不能形成有效实时障碍，定向雷达的可见性限制仍需考虑。

局部路径在0.20 m硬碰撞余量之外预留0.35 m跟踪空间，覆盖解析连接、格点搜索和缓存复核。只有精确起点或终点无法容纳额外跟踪空间时，才在相应端渐减额外空间；硬碰撞余量始终保留。跟踪偏差同时考虑位置和朝向对长车体的影响，偏差过大重新规划。MPPI沿路径线段投影进度，分别处理车头方向和行驶方向，换挡前制动至静止。时序相关的采样扰动、转向反转惩罚和角加加速度代价共同抑制左右摇摆，且加权结果重新做碰撞检查。

修改源配置后需要重新构建，运行时加载的是 `install/clearing_robot_plugins/share/clearing_robot_plugins/config/navigation.yaml`。

三个规划插件分别位于 `src/paper_global_planner.cpp`、`src/hybrid_local_planner.cpp`、`src/mppi_trajectory_planner.cpp`，并有各自的同名头文件；`planner_utils.hpp` 只存放公共路径转换等工具。

## 导航流程与速度平顺性

RViz或任务系统通过框架标准 `/navigate_to_pose` Action直接向 `bt_navigator` 下发原始目标位姿。行为树依次调用全局规划、局部规划、轨迹执行和恢复服务器，由各服务器加载清障机器人插件；自动选图及生命周期管理沿用MultiMapNav流程。

前进、倒车和转向由 `PaperGlobalPlanner`、`HybridLocalPlanner` 与 `MPPITrajectory` 根据目标位姿、运动学和碰撞约束处理。路径保留目标指定的车头朝向；目标在后方且朝向与当前一致时，插件可直接规划倒车路径。目标要求改变朝向时，由插件搜索满足该朝向的机动路径。到达判定继续使用行为树配置的位置、朝向容差及停车条件。

`motion` 是三种模式共同的运动约束来源：

| 参数 | 当前值 |
| --- | --- |
| 前进/倒车速度 | 0.35 / 0.20 m/s |
| 最大角速度 | 0.12 rad/s |
| 线/角加速度 | 0.18 m/s² / 0.08 rad/s² |
| 线/角加加速度 | 0.40 m/s³ / 0.15 rad/s³ |
| 规划最小转弯半径 | 3.5 m |
| 控制曲率半径基准 | 2.5 m，保留跟踪修正空间 |
| 横向加速度 | 0.08 m/s² |

框架的无时间戳 `Twist` 请求按实际仿真时钟采样，由现有C++速度保护器根据已执行状态校验速度、加速度和加加速度；同一仿真时刻的回调只更新待校验指令，不重复积分；合法规划指令直接执行，故障时沿受限曲线制动。暂停步进使用的 `TwistStamped` 保留其生成时间与确认协议。

末级速度链为规划/恢复 `/clearing/cmd_vel_raw` → 全车安全检查与S形速度整形 → 唯一 `/cmd_vel` 发布者 → 履带驱动。普通停车、目标切换和原始指令超时均沿有限加速度、有限加加速度曲线制动；驱动节点也保留自己的S形速度限制。`/clearing/cmd_vel_smooth` 与 `/clearing/cmd_vel_applied` 提供带仿真时间戳的整形指令和实际轮速换算指令，便于测量变化率。当前车体已进入碰撞包络或时钟回退时允许紧急归零，不以舒适性限制阻止紧急停车。

由收臂URDF惯性及平行轴定理生成的整车质量为29208 kg，绕总质心的偏航惯量约106681 kg·m²；控制预测检查两侧驱动力/偏航力矩、轮速耦合和横向加速度。每侧保守驱动力预算37500 N，使用2.8 m物理履带间距、3.276 m等效转向间距和0.48 m轮半径。这里采用低速平面动力学约束，土壤剪切、滑移与牵引参数尚未做实机辨识。

## 地图显示与固定视角

导航和建图的RViz均将 `Global Options / Fixed Frame` 和 `Views / Current / Target Frame` 设为 `map`，相机固定在地图参考系中。若后者改为 `base_link`，镜头会跟车移动，使整幅地图相对屏幕运动；仅设置Fixed Frame不能消除相机跟随。

当前帧雷达的可见范围会随运动变化，这与固定地图的整体平移不同。运动中的地图几何、TF和物理参考验证见[地图稳定性记录](map_stationarity_validation.md)。已打开的旧RViz需要重新加载配置，或手动修改Target Frame为map。

## 定位和 TF

FAST-LIO2 发布连续局部里程计，桥接为 `/odom` 和 `odom -> base_link`。五路 CustomMsg 进入 FAST-LIO2 前按同一盒体模型过滤机械臂/车体回波，按不超过0.05 s的时间差同步组合，在 IMU 坐标系输出；FAST-LIO2 使用单位外参和360°视场。这样避免仅前向单雷达在侧向几何约束不足时漂移。此数据流只依赖刚体外参，不依赖 FAST-LIO2 输出，因此没有融合输入的里程计循环依赖。用于局部感知的五路 PointCloud2 则在各采集时刻变换到 odom，再对齐到共同 base_link 时刻。

地图定位插件以 FAST-LIO2 为初值，对静态先验做 NDT/ICP 匹配，并检查收敛、重叠度、匹配分数、修正幅度和数据年龄。MultiMapNav 定位服务器发布唯一的 `map -> odom`，形成 `map -> odom -> base_link`。运行时不读取 Gazebo 真值位姿或真值里程计。

`/clearing/localization_status` 报告匹配质量与拒绝原因，`/clearing/localization_healthy` 控制运动许可。地图匹配用于约束累计漂移，不能保证绝对零漂移；无有效地图约束或环境退化时选择停止/重新初始化。

## 地图和适用范围

默认先验来自 `clearing_enclosed.world` 静态碰撞几何，保存在 `maps/`，来源哈希见 `provenance.json`。可重新生成：

```bash
python3 src/clearing_robot_plugins/scripts/prepare_assets.py --output src/clearing_robot_plugins
```

更换世界时必须提供相匹配的 `map_file`、`support_file`。全局论文算法是单高程层方法，不能将重叠楼层直接压成一层后宣称可规划；选择器分析完整三维先验，支撑层独立保存。

前进限速0.35 m/s、倒车0.20 m/s、角速度0.12 rad/s，预测采用6 s时域。爬坡上限约 12°，侧倾上限约 1.15°；侧倾采用低阈值是因为当前多轮滑移近似横向摩擦仅为 0.03。它们是仿真等效参数，实际履带、坡地和土壤需重新辨识。定向雷达仍存在未覆盖区域，已知先验和实时回波不能视为完整环境感知保证。

## 验证

```bash
bash src/multimap_nav_simulation/scripts/build_livox_workspace.sh --packages-select clearing_robot_plugins multimap_nav_simulation --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/clearing_robot_plugins/test/test_mode_configuration.py
ctest --test-dir build/clearing_robot_plugins -R clearing_geometry_test --output-on-failure
python3 src/clearing_robot_plugins/test/test_lidar_fusion.py
python3 src/clearing_robot_plugins/test/test_motion_self_filter.py
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase startup --forced-mode auto
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase navigate --forced-mode costmap2d
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase navigate --forced-mode gridmap
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase navigate --forced-mode octomap
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase navigate
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase chassis
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase turn
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase obstacle
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase sensor_loss
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase relocalize
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase recovery
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase visual
```

测试默认选择独立 ROS 域与空闲 Gazebo 主端口，也可用 `--domain`、`--gazebo-port` 指定；只停止自己创建的进程。物理状态服务只供测试误差断言，绝不接入定位或控制。`visual` 阶段需要 X11，用于检查真实 RViz 主窗口。本轮18盒体、前后换向与速度平顺性结果见 [运动修正验证](motion_refinement_validation.md)。较早的选图版本记录见 `map_modes_validation.md`；固定GridMap版本记录保留在 `validation.md`。

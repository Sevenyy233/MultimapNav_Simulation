# 基础几何体清障机器人

适配 ROS 2 Humble + Gazebo Classic。模型以 `clearing_robot.xacro` 的原始 CAD 尺寸、
各 link 坐标系和七轴关节安装关系为参考，运行时全部使用 box/cylinder。

```bash
cd /data/projects/ros2_ws/MultiMapNav_Simulation
source /opt/ros/humble/setup.bash
bash src/multimap_nav_simulation/scripts/build_livox_workspace.sh
source install/setup.bash
ros2 launch multimap_nav_simulation clearing_robot_primitives.launch.py
# 无界面运行：追加 headless:=true
```

## 尺寸与外形

保留原模型的低矮长平台、后悬、长中臂、折弯小臂、腕部和宽铲斗，
没有驾驶室或高位雷达塔。机械臂采用原关节限位内的前置低位行驶姿态，臂段长度保持原尺寸。

| 部件 | 原始 CAD 的局部 XYZ 包围尺寸 / m |
| --- | --- |
| 回转平台 link1 | 5.000 × 2.800 × 1.013 |
| 大臂 link2 | 2.788 × 0.780 × 0.776 |
| 中臂 link3 | 4.700 × 0.724 × 0.848 |
| 折弯小臂 link4 | 3.130 × 1.767 × 0.839 |
| 第一腕部 link5 | 0.801 × 0.524 × 0.621 |
| 第二腕部 link6 | 0.519 × 0.576 × 0.640 |
| 铲斗 link7 | 2.500 × 0.732 × 1.285 |

这些是部件自身坐标系的包围尺寸。大臂、中臂关节间距分别为 **2.5 m、4.5 m**，
小臂至腕部关节间距约 1.87 m。七个 arm joint 的 origin、rpy、axis 和父子关系与原模型一致，
同一关节姿态下可直接与原模型叠加对比。

底盘与履带名义占地约 **4.96 × 3.19 m**；低位行驶姿态包含后悬与前置机械臂，
整车包络约 **8.98 × 3.19 × 4.13 m**。铲斗底部离 base_link 的名义地面约 **0.55 m**，
铲斗顶部约 1.13 m；保留全尺寸机械臂时，行驶姿态仍有前悬。导航保守 footprint 在
`config/clearing_robot_primitives.yaml` 的 `robot_geometry.ros__parameters.navigation_footprint` 中给出，范围 x=[-3.45,6.05]、y=[-1.75,1.75] m。
总质量约 **29.2 t**，接近原模型的约 29.2 t。

主体和机械臂的长方体组合由原 CAD 外包络离线拟合得到：机械臂采样网格不超过 0.09 m，
底盘不超过 0.11 m，铲斗为 0.065 m。细连杆额外使用连续圆柱体。
小孔、倒角、紧固件和内部空腔有简化，因此这仍是导航仿真用的几何近似。
惯性沿用原模型已有的合理化 CAD 外包络惯量；500 kg 驱动轮按实心圆柱计算。
原底盘质量中的 400 kg 分配给两侧固定履带外观，避免重复计入质量。

离线拟合工具为 `scripts/generate_clearing_primitive_geometry.py`，它读取原 STL 仅用于生成数值参数。
启动使用已保存的 `robot/part/clearing_arm_geometry.xacro`，不会加载 STL。

```bash
# 可选：从原参考网格重新生成 box/cylinder 参数，只需 NumPy。
python3 src/multimap_nav_simulation/scripts/generate_clearing_primitive_geometry.py
```

## 收臂姿态与关节限位

控制器、速度桥、机械臂初值与几何限制统一存放在 `config/clearing_robot_primitives.yaml`。收起角度位于：

```yaml
initialize_clearing_arm:
  ros__parameters:
    positions: [0.0, 0.10, -0.03, 1.54, 0.0, -1.50, 0.0]
```

顺序为 joint1..joint7，单位 rad。此配置同时用于 GazeboSystem 的位置初值和启动时发送的保持轨迹。
关节零位保留原始 CAD 定义；零位下机械臂仍是原模型的伸臂外形。

七个关节的角度限位均与原 `clearing_robot.xacro` 一致，已撤销此前为反折而增加的范围：

| 关节 | 当前角度范围 / rad |
| --- | --- |
| joint3 | [-1.6895, 0] |
| joint4 | [0, 1.6265] |

机械臂朝前下方收拢，铲斗离地保持；没有把机械臂反折到平台上方。
全尺寸臂的前悬已计入导航包络，导航配置应使用该完整包络。
静态质心约为 base_link 中的 `(0.369, 0.013, 1.130)` m，其投影位于履带支承范围内。
这些检查覆盖原 URDF 行程、几何间隙和静态支承；液压连杆与执行器动态仍采用简化模型。

启动顺序：等待 Gazebo 时钟、生成机器人、激活关节状态和机械臂控制器、
确认收臂 action 成功、最后启用履带控制器与 `/cmd_vel` 速度桥。
轨迹结束后 `arm_controller` 持续保持最终姿态，直到用户下发新轨迹。

机械臂使用 GazeboSystem 理想位置接口，关节力矩限额 12–350 kN·m，
不包含液压执行器动态模型。初值与未被控制器接管时的保持行为见
[Gazebo ros2_control 官方文档](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)。

## 履带、五个 Avia 与 FAST-LIO2

左右各五个圆柱驱动轮，半径 **0.48 m**、宽 **0.30 m**、中心距 **2.8 m**，
安装位置与原模型一致。长方体上下条带与圆柱端部组成履带外观。
固定条带没有接地碰撞，物理驱动由十个旋转圆柱产生，这是多轮滑移转向近似。

差速关系：`B_eff=B*steering_separation_multiplier`，`left=(v-w*B_eff/2)/r`、`right=(v+w*B_eff/2)/r`。
实体半径 `r=0.48 m`、间距 `B=2.8 m` 从控制 YAML 同时提供给 xacro 与速度桥；
补偿系数默认 `1.17`，只改变左右速度差，不改变实体轮距。该系数来自当前封闭平地场景的低速标定。
速度桥只换算速度、不积分位姿，交给 ros2_control 的
`velocity_controllers/JointGroupVelocityController`，见
[Humble 官方控制器接口](https://control.ros.org/humble/doc/ros2_controllers/velocity_controllers/doc/userdoc.html)。
横向摩擦默认 0.03，纵向 1.2；可通过 `lateral_friction` 调整滑移转向响应。

| 接口 | 内容 |
| --- | --- |
| `/cmd_vel` | geometry_msgs/Twist，使用 linear.x 和 angular.z |
| `/tracks_velocity_controller/commands` | 十个轮速度，先五左后五右，rad/s |
| `/arm_controller/follow_joint_trajectory` | 七轴机械臂轨迹 action |
| `/joint_states` | 轮关节和机械臂关节反馈 |
| `/odom` | FAST-LIO2 位姿转换到 base_link，父坐标系 odom |
| `/fast_lio/odometry` | FAST-LIO2 原生位姿，camera_init -> body（IMU） |
| `/tf` | 默认由定位桥发布 odom -> base_link |
| `/livox/lidar_1` … `/livox/lidar_5` | 五路独立 livox_ros_driver2/CustomMsg，10 Hz |
| `/livox/lidar_1_PointCloud2` … `/livox/lidar_5_PointCloud2` | 对应五路 PointCloud2，frame 为 rslidar_1 … rslidar_5 |
| `/livox/imu_raw` | 200 Hz IMU，m/s²，frame 为 imu_link |
| `/fast_lio/lidar` | 前左雷达的仿真时间适配输入，供 FAST-LIO2 使用 |

五个雷达均直接调用 `ros2_livox_simulation/urdf/avia.xacro` 的宏，使用其 Avia CSV 扫描轨迹，
不复制或修改雷达仿真包。启动文件为展开后的 Gazebo 传感器补齐 `always_on` 及同源触发射线配置，
确保无界面时也持续采样。每帧沿 24,000 点的原始轨迹片段推进，默认 `lidar_downsample:=8` 抽样为 3,000 个射线点，
采用 Avia 的前向视场（约 70.4° × 77.2°）。
可设 `lidar_downsample:=1` 恢复 24,000 点/帧，但五路全密度 CPU 射线会显著增加内存和运算开销。插件内部辅助网格及触发网格均为 2 × 2，实际点云仍由 CSV 轨迹产生。Gazebo 中的传感器射线显示全部关闭；`always_on=true` 保证关闭显示后五路点云仍持续发布。已移除此前用于扫描显示的 `lidar_visualization_range`、`lidar_visualization_3d` 参数。
每个雷达观测自身朝向的区域。前方两台外偏 ±10°，让70.4°水平视场在前方中线重叠；原 ±45°在外沿安装下留下明显中线盲区。移除了后方支杆及原单个导航雷达。
上游 PointCloud2 发布实现会将每个采样点写入两次，默认因此有 6,000 条记录；CustomMsg 为 3,000 点。

下表坐标均相对 `link1`，rpy 中仅 yaw 非零。保留后、左、左前、右前、右的分工，将左、右及前方四个安装点移到平台外沿；原位置向下的射线被平台/履带遮挡，实测没有地面回波。移动后的射线可以直接到达地面：

| 雷达 | 方向 | XYZ / m | yaw / rad |
| --- | --- | --- | --- |
| rslidar_1 | 后 | (-3.2, 0, -0.6) | pi |
| rslidar_2 | 左 | (-0.5, 1.55, -0.6) | pi/2 |
| rslidar_3 | 前左 | (2.0, 1.55, -0.6) | pi/18 |
| rslidar_4 | 前右 | (2.0, -1.55, -0.6) | -pi/18 |
| rslidar_5 | 右 | (-0.5, -1.55, -0.6) | -pi/2 |

IMU 固定到同一 `link1` 的 `(0,0,-0.663)`，中立回转姿态下对应 base_link 中的 `(0,0,1.65)`。
因此上车体回转时雷达与 IMU 的相对位姿仍固定。五个 Avia 的固定关节均在 Gazebo 中保留，
防止固定关节合并引起插件碰撞对象重名或传感器坐标错误。

默认在收臂成功、履带控制器激活后启动 FAST-LIO2。上游算法仅接收一路雷达，
这里使用前左 `rslidar_3`，另外四路保留独立点云，供后续环境感知使用；没有直接拼接不同坐标系的扫描。
外参保存在同一份 `config/clearing_robot_primitives.yaml`，为 lidar -> IMU 的
`T=[2.0,1.55,0.063]`、`R=Rz(pi/18)`，与模型实际安装关系一致。

`fast_lio_sim_bridge` 只适配输入时间和输出坐标：雷达插件每帧对冻结的物理场景采样，
原 `offset_time` 实际记录 CPU 墙钟耗时，不能用于慢速仿真的运动去畸变。
桥将该瞬时扫描内各点时间设为帧的 Gazebo 时间（offset=0），保留五路原始消息，
并将 FAST-LIO2 的 IMU 位姿结合实际关节 TF 转成 base_link 位姿。
原生 FAST-LIO2 TF 隔离到 `/fast_lio/tf`，避免给同一机器人发布第二套父子关系。
主 TF 树使用 odom -> base_link -> link1 -> imu_link -> body，odom -> camera_init 为恒等变换。
`odom` 原点沿用 FAST-LIO2 初始化时的 IMU 原点，不与 world 强制重合。
FAST-LIO2 上游未提供有效速度，桥对变换后的底盘位姿按消息时间求差，在 `base_link` 坐标系发布线速度和角速度。
`twist_filter_time_constant=0.2 s` 用于平滑速度；首帧、重复/倒退时间戳或超过 `twist_max_interval=0.5 s` 的采样间隔会重置估计，避免产生时间跳变尖峰。
速度不是由 `/cmd_vel` 或轮速代替，也不使用物理真值。速度来自同一套定位位姿，不是独立传感器测量。
桥未传播位姿杠杆臂及差分速度的统计协方差，两者仍标为未知（对角线 1e6）。

最大线速度 1.0 m/s、角速度 0.35 rad/s；仿真时间下的加速度限制分别为 0.3 m/s²、0.2 rad/s²。
连续 0.5 秒真实时间未收到指令，速度桥下发零速。

`base_link` 和原模型一样位于名义地面附近，轮最低点为 z=-0.01 m。
不使用轮式里程计控制器，也不积分轮速。此模型不加载 Gazebo P3D 真值里程计插件，
启动文件不再包含真值 TF 桥或 `publish_odom_tf` 参数。
默认仅发布 `/fast_lio/odometry` 和适配后的 `/odom`，主 TF `odom -> base_link` 来自 FAST-LIO2。
关闭定位可追加 `use_fast_lio:=false`；此时本入口不发布 odom 或 odom -> base_link。

## 底盘导航参数核对

当前模型用于机械臂固定收起时的低速滑移转向导航。实体质量、惯量与接触参数保留，导航执行采用带等效转向补偿的运动学映射。

| 项目 | 当前值及核对结果 |
| --- | --- |
| 整机质量 | 29,208 kg；各惯量正定且满足主惯量三角关系 |
| 收臂静态质心 | base_link 中约 (0.370, 0.013, 1.130) m，投影在轮接触支承区域内 |
| 实体轮半径 / 左右间距 | 0.48 / 2.8 m，模型与驱动统一使用 YAML |
| 最大底盘线 / 角速度 | 1.0 m/s、0.35 rad/s |
| 线 / 角加速度限制 | 0.3 m/s²、0.2 rad/s²，按仿真时间限制普通指令变化 |
| 单轮速度上限 | 8 rad/s，与模型关节限位一致；最高底盘组合指令低于此上限 |
| 指令超时 | 0.5 秒真实时间，超时立即下发零轮速，属于失联保护 |
| 导航包络 | 保留收臂整车约 9.5 × 3.5 m 的保守矩形，包含机械臂前悬 |
| 接触与转向 | 纵向摩擦 1.2、横向等效摩擦 0.03；转向补偿系数 1.17 |

0.03 的横向摩擦和 1.17 的转向补偿属于十轮接触近似在当前平地场景的等效参数，不是实车履带/土壤辨识值。
标定同时检查直行、倒车、左右原地转向及左右圆弧；弧线仍有横向滑移，应由路径跟踪反馈处理，底盘不具备独立横移控制。
不同地面材质、坡度或机械臂工作姿态需要重新验证，当前结果只覆盖本配置的低速收臂行驶。

2026-10-27 底盘参数复测：

| 稳态指令 | 修改前实测 | 修改后实测 |
| --- | --- | --- |
| 直行 0.4 m/s | 0.404 m/s | 0.404 m/s |
| 倒车 -0.4 m/s | -0.396 m/s | -0.396 m/s |
| 原地左转 0.20 rad/s | 0.172 rad/s | 0.203 rad/s |
| 原地右转 -0.20 rad/s | -0.172 rad/s | -0.203 rad/s |
| 左圆弧 0.4 m/s、0.15 rad/s | 0.127 rad/s | 0.150 rad/s |
| 右圆弧 0.4 m/s、-0.15 rad/s | -0.127 rad/s | -0.150 rad/s |
| 最高直行 1.0 m/s | 1.004 m/s | 1.004 m/s |

转向标定由物理状态服务离线测量完成，运行中的速度反馈仍完全来自 FAST-LIO2。
默认 FAST-LIO2 运动回归实测 `/odom` 前进速度约 0.405 m/s、转向角速度约 0.202 rad/s，
停车后速度回落；五路地面回波和机械臂保持/轨迹回归通过。
4 项速度估计单元测试、13 项模型检查、3 项入口布局检查通过，
原始采样、前后对比和构建日志保存在 `log/chassis_parameter_audit_20261027/`。


## 当前工作空间依赖与构建

当前工作空间包含 `fast_lio`、`ros2_livox_simulation`、`livox_ros_driver2`、`multimap_nav_simulation`。
缺失的 `livox_ros_driver2` 从 [Livox 官方仓库](https://github.com/Livox-SDK/livox_ros_driver2) 下载到本工作空间的 `src/`。
Livox-SDK2 已安装于系统 `/usr/local`，其余 Humble、PCL、Eigen、Protobuf、Boost、APR 依赖经 rosdep 检查满足。
不使用 Lidar_nav2_ws 或其他工作空间的安装结果。

上述构建脚本会清理继承的 ROS 前缀，只叠加系统 `/opt/ros/humble`，串行编译当前 `src/` 中的包。
脚本仅选择 driver2 的 ROS2 manifest，不调用会删除整个 build/install 的上游 build.sh。
`cmake/vendor_compat.cmake` 在构建阶段把雷达包写死的 Protobuf/Boost 旧库名替换为系统实际库路径，
不创建假 ABI 软链接；也将 FAST-LIO2 的 ROOT_DIR 指向 `build/fast_lio/runtime/`，
避免其无条件创建的调试日志写入源码目录。两个现有上游包均保持源码不变。
后续重编译请使用这里的构建脚本，避免丢失构建适配。

## 场景与启动参数

`worlds/clearing_robot/clearing_enclosed.world` 内部净空 **50 × 50 × 20 m**，
x/y=[-25,25]、z=[0,20] m。地板、四面墙、顶盖均有碰撞，顶盖显示为半透明。
四个箱体、两个圆柱、两条短隔墙提供导航观测特征，中心出生区留空。
地面接触刚度 kp=1e9、阻尼 kd=1e5，适配重型机器人。

出生默认 `(0,0,0.03,yaw=0)`，接地后的 base_link 高度约 0.009 m。
可使用 `spawn_x/spawn_y/spawn_z/spawn_yaw`、`gui/headless` 和 `use_fast_lio`。
`world_file:=/absolute/path/example.world` 可替换世界，出生高度应匹配地面。

```bash
ros2 control list_controllers
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.4}, angular: {z: 0.0}}'
```

## 验证

```bash
python3 src/multimap_nav_simulation/test/test_clearing_robot_primitives.py
python3 src/multimap_nav_simulation/test/test_robot_layout.py
python3 src/multimap_nav_simulation/scripts/validate_map_suite.py
python3 src/multimap_nav_simulation/test/smoke_clearing_robot_primitives.py
```

静态检查覆盖几何体、惯量、关节安装关系、原始限位、收臂包络、机械臂间隙、质量和地图，
并核对五个雷达的朝向、安装点相对车体碰撞包络的外侧间隙、FAST-LIO2 外参与 IMU 安装关系。
运行测试使用独立 ROS_DOMAIN_ID=173 和 Gazebo 11373 端口，可预先通过环境变量覆盖。
结果与日志写入 `/tmp/clearing_primitives_smoke_*/`，只关闭测试自身创建的进程。
测试覆盖五路 CustomMsg 与 PointCloud2、各自前向视场和真实地面回波、IMU、FAST-LIO2 位姿和 TF、
直行定位增量与物理位姿对比、转向、指令超时停车、行驶期间收臂保持及七轴轨迹返回。
评估脚本只通过 Gazebo `get_entity_state` 服务查询物理位姿作断言参考，不添加任何真值 odom/TF 发布器，定位节点不使用这些查询。

2026-10-27 验证：重新构建通过，12 项清障模型检查、3 项入口布局检查及完整无界面运动回归通过。
移位前地面回波只有后雷达有约 1,081 点/帧，其余四路为 0；移位后单帧 CustomMsg 抽样的
五路地面点数分别为 1,101、1,046、991、578、1,049（每路总计 3,000 点）。
地面判定使用物理世界 z=0 附近 ±0.06 m 的真实点云；不是添加合成地面。
关闭 FAST-LIO2 时没有 Odometry 话题；默认启动时仅有 `/fast_lio/odometry` 和 `/odom`，
`/odom` 只有一个发布器，odom -> base_link 来自定位桥。
直行、转向、指令超时停车和机械臂保持/轨迹测试通过；这些检查不代表复杂环境下的定位精度评测。
构建、前后地面回波及运行回归结果保存在 `log/lidar_ground_fix_20261027/`。

`navigation_footprint` 保存为 JSON 字符串，以兼容 ROS 2 参数 YAML；读取几何时解析为二维顶点。

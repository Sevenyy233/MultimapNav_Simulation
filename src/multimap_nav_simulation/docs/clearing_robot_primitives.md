# 基础几何体清障机器人

适配 ROS 2 Humble + Gazebo Classic。模型以 `clearing_robot.xacro` 的原始 CAD 尺寸、
各 link 坐标系和七轴关节安装关系为参考，运行时全部使用 box/cylinder。

```bash
cd /data/projects/ros2_ws/MultiMapNav_Simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select multimap_nav_simulation --symlink-install
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
总质量约 **29,220.5 kg**，接近原模型的约 29.2 t。

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

## 履带、传感器与真值

左右各五个圆柱驱动轮，半径 **0.48 m**、宽 **0.30 m**、中心距 **2.8 m**，
安装位置与原模型一致。长方体上下条带与圆柱端部组成履带外观。
固定条带没有接地碰撞，物理驱动由十个旋转圆柱产生，这是多轮滑移转向近似。

差速关系：`left=(v-w*2.8/2)/0.48`、`right=(v+w*2.8/2)/0.48`。
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
| `/odom` | Gazebo P3D 三维真值，父坐标系 world |
| `/truth/odometry` | 真值的父坐标系转换为 odom，速度转换到 base_link |
| `/tf` | 默认发布真值 odom -> base_link |
| `/points_raw` | 360°、16 线、720 水平采样、10 Hz、最大 80 m 的 3D 点云 |
| `/livox/imu_raw` | 200 Hz IMU |

仅使用 P3D 真值里程计，没有轮式里程计控制器或轮式里程计话题。
IMU 安装在 base_link 的 `(0,0,1.65)`，导航雷达位于 `(-1.5,0,2.6)`，与原模型安装位置一致。
雷达和 IMU 固定在底盘上，其外参不随上车体回转。真实视线会被本体遮挡，
导航需要按完整收起包络配置 footprint，并过滤本体附近点云。

最大线速度 1.0 m/s、角速度 0.35 rad/s；仿真时间下的加速度限制分别为 0.3 m/s²、0.2 rad/s²。
连续 0.5 秒真实时间未收到指令，速度桥下发零速。

`base_link` 和原模型一样位于名义地面附近，轮最低点为 z=-0.01 m。
world 与 odom 原点重合。`publish_odom_tf:=false` 可关闭真值 TF 节点和 `/truth/odometry`，
`/odom` 仍由 P3D 发布。

## 场景与启动参数

`worlds/clearing_robot/clearing_enclosed.world` 内部净空 **50 × 50 × 20 m**，
x/y=[-25,25]、z=[0,20] m。地板、四面墙、顶盖均有碰撞，顶盖显示为半透明。
四个箱体、两个圆柱、两条短隔墙提供导航观测特征，中心出生区留空。
地面接触刚度 kp=1e9、阻尼 kd=1e5，适配重型机器人。

出生默认 `(0,0,0.03,yaw=0)`，接地后的 base_link 高度约 0.009 m。
可使用 `spawn_x/spawn_y/spawn_z/spawn_yaw`、`gui/headless` 和 `publish_odom_tf`。
`world_file:=/absolute/path/example.world` 可替换世界，出生高度应匹配地面。

```bash
ros2 control list_controllers
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.4}, angular: {z: 0.0}}'
```

## 验证（2026-10-16）

离线回归检查：

```bash
ctest --test-dir build/multimap_nav_simulation \
  -R 'clearing_robot_primitives_consistency|map_suite_consistency' --output-on-failure
```

检查纯基础几何体、正定惯量、原始关节安装关系、CAD 部件尺寸误差（每个包围边界不超过 0.11 m）、
原始角度限位、低位行驶包络、铲斗离地间隙、非相邻碰撞几何间隙、静态质心、控制配置和封闭世界尺寸，并回归原有 17 张地图。

完整 Gazebo 无界面测试（自动启停，仅关闭测试自身创建的进程）：

```bash
python3 src/multimap_nav_simulation/test/smoke_clearing_robot_primitives.py
```

脚本使用独立 ROS_DOMAIN_ID=173 和 Gazebo 11373 端口；可预先用环境变量覆盖。
结果与日志写入新的 `/tmp/clearing_primitives_smoke_*/` 目录。
检查前进、原地转向、速度超时停车、行驶期间收臂保持、七轴小幅轨迹及返回收臂，
以及 IMU、场景点云、真值里程计和 TF。此启动不自动运行导航算法。

当前低位行驶姿态的无界面实测结果：

| 项目 | 结果 |
| --- | --- |
| 前进 | 0.4 m/s 指令持续 6 s 仿真时间，位移约 2.15 m（包含加速段） |
| 原地转向 | 0.2 rad/s 指令持续 8 s，偏航变化约 1.28 rad（74°） |
| 行驶期间收臂保持 | 最大关节偏差约 7.6e-6 rad |
| 七轴小幅轨迹及返回收臂 | 两次 action 均成功，返回误差约 3.8e-6 rad |
| 速度超时 | 指令超时后十个驱动轮命令归零 |
| 雷达 | 11,520 个有限坐标点，其中 10,674 个点的距离超过 5 m |
| 真值、IMU 与 TF | 正常；里程计话题只有 `/odom` 和 `/truth/odometry` |
| 十项模型检查与原 17 张地图回归 | 通过 |

上述姿态精度对应 GazeboSystem 的理想位置接口，并非实机精度。

`navigation_footprint` 保存为 JSON 字符串，以兼容 ROS 2 参数 YAML；读取几何时解析为二维顶点。

# 清障机器人 Gazebo 仿真

模型从 `robot_description/urdf/whc_description_gazebo.xacro` 迁入本包，入口为
`robot/clearing_robot.xacro`，所需 STL 位于 `meshes/clearing_robot/`。
不再需要原来的 `robot_description` 功能包。适用于 ROS 2 Humble + Gazebo Classic。

## 构建与启动

```bash
cd /home/yangyang/ros2_ws/MultiMapNav_Simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select multimap_nav_simulation --symlink-install
source install/setup.bash
ros2 launch multimap_nav_simulation clearing_robot.launch.py
```

默认使用 `cabin.world`。`spawn_x/y/z/yaw` 留空时按世界选择默认出生位姿
（启动文件中的 `WORLD_SPAWN_POSES`）：

| 世界 | x, y, z, yaw |
| --- | --- |
| `cabin` | 0, 0, 0.05, 0 |
| `terrain` | 10, -4, 0.3, π（原点处有土丘，会把车体嵌进地形） |
| `terrain_with_wall` | 3, 9, 0.3, π |
| 其他 | 0, 0, 0.05, 0 |

本包安装 cabin 的 STL 和 Gazebo 模型目录，启动时自动设置模型搜索路径。
生成机器人前等待 `/clock` 连续前进，避免保存过仿真状态的世界加载时钟跳变
导致 Gazebo 生成服务误判超时。

```bash
# 切换本包的世界（名称可带 .world）
ros2 launch multimap_nav_simulation clearing_robot.launch.py world_file_name:=terrain

# 自定义世界文件、出生位置，以及无界面模式
ros2 launch multimap_nav_simulation clearing_robot.launch.py \
  world_file:=/absolute/path/example.world spawn_x:=6.5 spawn_y:=-3.5 headless:=true
```

`gui:=false` 也可关闭界面。`world_file` 非空时优先于 `world_file_name`。
这个机器人宽约 3.3 米、底盘长约 5 米。零位时机械臂往前伸出约 7.8 米、最高约 5.2 米。

`stiff_ground:=true`（默认）时，若世界文件里定义了接触 `kp/kd`，启动文件会
在系统临时目录生成一份把它们改为 `1e9/1e5` 的副本并加载，原世界文件不变。
`terrain`、`terrain_with_wall`、`two_floor` 为小车设置了 `kp=1e6`，Gazebo 按
串联合成接触刚度，约 29 吨的本机器人会下陷约 10 cm，行驶后穿过地形网格坠落。

## 机械臂

`joint1` 到 `joint7` 仍是原来那 7 个转动关节，安装位姿与
`whc_description_gazebo.xacro` 一致。启动时 `arm_controller`（轨迹控制器）把它们
保持在 URDF 零位，否则重力会把臂拉下去。需要活动时向
`/arm_controller/follow_joint_trajectory` 发目标即可。

## 仿真动力学与碰撞近似

保留原模型的视觉网格和各链接质量；STL 链接的惯量按均匀密度凸包近似重算，
车轮惯量按实心圆柱计算。原底盘 STL 包含贴地的刚性履带外形，直接作为碰撞
网格会拖地，因此底盘碰撞改用抬高的简化箱体，由十个驱动轮承担地面接触。
转台和机械臂的碰撞体使用各 STL 的包围盒（视觉仍为原网格）：原 STL 作为碰撞
体会让 gzserver 在雷达射线检测中段错误崩溃。
这些设置用于底盘导航仿真，不精确复现液压机械臂或履带的接触动力学。

## 控制与传感器接口

| 接口 | 用途 |
| --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` 底盘速度 |
| `/odom` | Gazebo 三维真值里程计 |
| `/odom_diff_drive` | 差速轮式里程计 |
| `/joint_states` | 十个车轮和七个机械臂关节 |
| `/points_raw` | 导航用 3D 点云，坐标系 `lidar_link` |
| `/livox/imu_raw` | IMU，坐标系 `imu_link` |
| `/rslidar_points_1` 至 `/rslidar_points_5` | 原模型的五个周向雷达 |

默认发布 `odom -> base_link` 真值 TF；与定位系统一起使用时，若该系统也发布
同一 TF，可设置 `publish_odom_tf:=false`。
控制器参数在 `config/clearing_robot_controllers.yaml`。

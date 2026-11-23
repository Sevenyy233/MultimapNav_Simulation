# 清舱机器人导航

本包接入 MultiMapNav 的 `GlobalPlannerBase` 和统一 `LocalControllerBase`。地图选择器分析同一 PCD，生命周期管理器加载选中模式的地图、定位、全局 A* 和局部控制插件。全局服务器接收目标并生成 `nav_msgs/Path`；局部服务器调用 `computeLocalPath` 和 `computeVelocityCommands`，向履带速度桥发布 `/cmd_vel`。训练模式只激活地图、定位及全局规划服务器，`/clearing/training_control` 持有运动命令并提供同一 C++ 局部算法。

## 构建和运行

ROS 2 Humble、Gazebo Classic，先加载 MultiMapNav 工作空间。传统模式只需 ROS 依赖。要启用 C++ ONNX 推理，准备 Linux x64 ONNX Runtime C++ SDK，把 `ONNXRUNTIME_ROOT` 指向包含 `include/onnxruntime_cxx_api.h` 和 `lib/libonnxruntime.so` 的目录。SDK 安装后请保留该目录供加载动态库。[ONNX Runtime C++ SDK](https://onnxruntime.ai/docs/get-started/with-cpp.html)

```bash
source /opt/ros/humble/setup.bash
source /data/projects/ros2_ws/MultiMapNav/install/setup.bash
cd /data/projects/ros2_ws/MultiMapNav_Simulation
colcon build --symlink-install \
  --packages-up-to clearing_robot_drl \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DONNXRUNTIME_ROOT=/path/to/onnxruntime
source install/setup.bash
```

在两个终端分别启动仿真和导航。两个命令使用默认 ROS 域，Gazebo GUI 和 RViz2 默认开启。

```bash
ros2 launch clearing_robot_simulation clearing_robot.launch.py
ros2 launch clearing_robot_plugins navigation.launch.py
```

RViz 默认使用 `Orbit` 视角。左键拖动旋转、鼠标中键拖动平移、滚轮缩放。切换地图通过 **Navigation Mode** 面板选择 `auto`、`costmap2d`、`gridmap` 或 `octomap` 并点击 **Apply**，也可以执行：

```bash
ros2 param set /clearing_navigation_modes map_mode gridmap
```

切换会取消当前导航目标，等待底盘停止，再装载新模式的地图和导航节点。等待面板显示 **Active map** 后，使用 **Nav Goal Tool** 下发目标。Gazebo、FAST-LIO2、RViz2 和所选相机视角持续运行。三个全局地图显示项采用 MultiMapNav 原版 `auto_nav.rviz` 的颜色及显示参数，分别订阅 `/costmap`、`/grid_map` 和 `/octomap_map`。面板自动显示选中的地图；规划路径使用绿色全局线与品红局部线。可在启动时通过 `map_mode:=gridmap` 等参数指定初始模式。

部署已导出的策略时指定 `controller:=drl onnx_model:=/path/to/policy.onnx`。训练导航使用 `controller:=training map_mode:=gridmap`。独立评估环境由训练程序按需启动及回收。训练、评估和记录说明见 [clearing_robot_drl](../clearing_robot_drl/README.md)。

## 规划和控制约束

三个 A* 共用带航向的状态格、前向直线和定曲率圆弧。部署配置使用折叠运输包络 `x ∈ [-3.45, 6.05] m`、`y ∈ [-1.75, 1.75] m`、最大高度 4.30 m，并留 0.20 m 平面余量。候选边逐片采样整车扫掠，校验支承面、坡度、台阶和障碍。最小转弯半径 4.2 m，通过 `v ≤ sqrt(a_lat / |κ|)` 限制曲线的规划速度代价。航向改变量提供启发式下界，弯曲、曲率切换、坡度增加路径代价。近目标使用前向 Dubins 连接并复核车体，保持请求的 XY 和航向。地图边界和未知支承面按不可通行处理。Costmap2D 读取平面高度和坡度，GridMap 读取高程、坡度和通行层，OctoMap 查验支承体素和上方占据。全局输出是几何参考；局部控制负责速度执行、急停、当前点云动态避障。

MPPI 对滚动参考路径采样受限控制序列，用指数代价权重求首个速度指令。整车包络、地形、跟踪误差、前进效率及转向变化进入优化。跟踪模型按差速车体系积分及 IMU/LIO 反馈估计的牵引折减滚动，前向巡航命令 0.50–1.00 m/s，停车命令为零，角速度上限 ±0.12 rad/s，同时受最大曲率限制；线、角加速度上限 0.30 m/s² 和 0.20 rad/s²。仿真底盘进一步实施加加速度限值。部署策略以与训练相同的输入生成归一化动作，之后执行同一安全检查。动作不可行时回退已检查的 MPPI 命令或停车。

## 定位和局部地图

五路 Avia `/livox/lidar_1..5` 的刚体坐标 `rslidar_1..5` 与 IMU 均安装于同一关节连杆。融合使用 URDF/TF 到 `imu_link` 的实测配置外参，扫描时间使用 Gazebo 时间，默认整组相差 ≤40 ms。Avia 仿真插件给出的单点时差是 CPU 耗时，融合点只保留对应整帧在组内的采集时差。过滤小于 0.30 m、大于 55 m 和机体自反射的点，按纳秒偏移排序后供 FAST-LIO2 使用。IMU 话题是 `/livox/imu_raw`。FAST-LIO2 在 IMU 坐标系使用单位旋转、零平移，输出桥接后的 `/odom` 与去畸变点云 `/points_raw`。原始 FAST-LIO2 里程计不包含先验地图绝对位姿。

定位复用新版 MultiMapNav 的 PointCloudLocalization。启动参数作为候选位姿，启用全图搜索和多帧匹配确认；定位丢失后重新搜索候选。定位健康状态约束地图发布和运动。回合复位时先停车与移位，清空近场动态点并重建 FAST-LIO2，相同的地图匹配条件确认新位姿。复位期间 Gazebo 时钟持续推进。

`/clearing/local_map` 同步携带 `elevation`（米）、`slope`（弧度）和障碍占据八叉树。两层按同一有效扫描时间在 `map` 坐标系更新，XY 轴对齐，以机器人为中心各伸展 ±5 m。OctoMap 的 Z 窗口从车底向上 10 m，边界裁剪在发布前完成，只保留完整位于窗口内的体素。代价与支承高度通过独立的控制感知窗口计算，涵盖前伸 6.05 m 的收拢机械臂、全车占据及前视制动区域。`/clearing/local_map/terrain` 和 `/clearing/local_map/occupancy` 发布同一复合消息的两部分，供标准 RViz 插件分别加载。

RViz 的 **Local Terrain and Occupancy** 可分别开关地形和占据层。地形可选择高度层与着色层、色带、数值范围和透明度；占据支持颜色、高程着色、透明度及 Octree 深度。先验地形保留；实时动态占据默认 2 秒过期。定位不健康或估计器复位会清空实时缓存。未知高程与坡度为 NaN。

训练碰撞同时读取 Gazebo 非支承接触与零余量包络占据相交，两者任一触发均结束回合；倾覆由姿态阈值单独记录。仿真底盘与整理后的点云仍需在典型侧滑、接触与遮挡工况下定量标定。

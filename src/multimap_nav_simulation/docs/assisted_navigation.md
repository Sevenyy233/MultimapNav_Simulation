# tracked_robot 仿真与辅助定位启动

source ROS、MultiMapNav_Simulation/install/setup.bash 与 MultiMapNav/install/setup.bash 后，用一条命令按相同地图启动仿真和导航：

```bash
ros2 launch multimap_nav_simulation gazebo_tracked_sim.launch.py world_file_name:=flat_2 navigation:=true
```

地图可换为 flat_1..5、floor_1..5、terrain_1..5；上层出生添加 spawn_level:=upper。仿真包自动选择对应PCD、经过碰撞几何核验的初始位置和高度，加载真值辅助相对里程计配置，并关闭重复的轮式里程计TF桥。

分开启动时仍可使用原仿真命令，待 tracked robot ready 后，导航使用：

```bash
ros2 launch multimap_nav_simulation tracked_navigation.launch.py world_file_name:=flat_2
```

`/odom`为P3D世界系参考状态，`/truth/odometry`提供车体系速度和相对运动。`/amcl_pose`与`/localization/odometry`仍由LiDAR、IMU和NDT–ESKF估计产生。通用bring_up不再默认把机器人初始化到原点；单独使用时必须提供正确初始位姿与里程计来源。

雷达最小量程为0.4m，覆盖当前tracked_robot的自体包络。`/obstacle_cloud`是当前真实雷达回波去除自体和地面后的点云，包含已知墙面，以base_link为坐标系，保留采集时间。`/dynamic_obstacle_cloud`保留为同一数据的兼容话题，不能据该名称判定物体正在运动。静态地图仅提供地形支撑与规划信息；默认不对障碍回波做静态地图差分。

RViz导航视图使用map固定坐标系。避障数据在连续odom坐标系保存，并与当前路径统一变换，过期数据按配置停车。当前履带仿真使用横向摩擦0.3和纵向摩擦6.0，导航精度结论受理想辅助里程计及已知地图条件限制。

# 仿真与导航分别启动

本包的四个机器人 launch 入口均只负责仿真：Gazebo、机器人、传感器、控制器及可选的里程计 TF。

仿真窗口：

```bash
source /opt/ros/humble/setup.bash
source /home/yangyang/ros2_ws/MultiMapNav_Simulation/install/setup.bash
ros2 launch multimap_nav_simulation tracked_robot.launch.py world_file_name:=flat_1
```

导航窗口加载导航工作空间后独立运行：

```bash
source /opt/ros/humble/setup.bash
source /home/yangyang/ros2_ws/MultiMapNav_Simulation/install/setup.bash
source /home/yangyang/ros2_ws/MultiMapNav/install/setup.bash
ros2 launch bring_up tracked_robot_nav.launch.py map_name:=flat_1
```

地图匹配、定位初值、算法模式和 RViz 设置归导航工程的 `bring_up` 包管理。履带导航参数已迁至 `bring_up/config/tracked_robot_nav_params.yaml`。仿真配置只包含仿真所需参数，仿真窗口不需要导航工作空间。

改变出生层时，两窗口分别传入 `spawn_level:=upper`；自定义出生位姿时，在导航入口显式设置对应 `initial_pose`。传感器话题、物理参数和地图资源保持原样。

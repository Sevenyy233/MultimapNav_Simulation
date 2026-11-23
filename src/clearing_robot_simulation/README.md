# 清障机器人仿真

`ros2 launch clearing_robot_simulation clearing_robot.launch.py` 启动训练场景；`world_file_name:=evaluation` 选择独立评估场景，`gui:=false headless:=true` 禁用图形界面，`use_fast_lio:=false` 关闭内置定位。默认出生位置为 `(0, 0, 0.03)`、偏航角为 0，可用 `spawn_x`、`spawn_y`、`spawn_z`、`spawn_yaw` 指定。

原基础几何模型及履带速度控制、机械臂折叠初始值和五台 Avia 与 IMU 保持原配置。控制器和速度转换节点按启动顺序加载。默认主雷达为 `/livox/lidar_3`、坐标系 `rslidar_3`；IMU 为 `/livox/imu_raw`。地图文件 `worlds/training.pcd` 和 `worlds/evaluation.pcd` 由各场景的静态三角地形和障碍碰撞面采样，其坐标轴及原点与 Gazebo world 一致；导航使用时令 map 与 world 对齐。场地约 120 × 100 m，起点附近平坦，前方连续起伏，地标、堆体和高障碍分布在中央可通行区域两侧；训练和评估采用不同地形波形及障碍布局。`worlds/training_goals.json` 与 `worlds/evaluation_goals.json` 给出同一区域的目标点和参考路线，z 值按碰撞三角形插值。`scripts/generate_worlds.py` 可重新生成场景、网格、配套点云及目标。

场景加载 `libgazebo_ros_state.so`，服务命名空间 `/gazebo`；正常仿真持续运行，无训练期间暂停物理的机制。此包复用 `multimap_nav_simulation` 中的 FAST-LIO 桥接和地图解析工具，因此仍需构建该包及 Livox 仿真依赖。五雷达融合由清舱导航功能包启动。

原 `worlds/clearing_robot` 和 `worlds/drl` 场景及其 PCD、网格和清单完整保存在本包 `worlds` 对应路径下；DRL 清单中的相对路径与校验和保持不变。`meshes/clearing_robot` 和 `3DModel/clearing_robot` 保留原始模型资源。`archive/legacy_clearing_robot.xacro` 是保留网格路径与原始参数的历史 STL 模型，依赖旧包共享的 `lidar_3d.xacro`，不作为新仿真启动入口；同目录留存其历史启动文件和配置，仅供追溯。

Gazebo 内的 ROS 插件默认加载 `config/gazebo_sim_time.yaml`，使 Avia 消息头的节点时钟使用仿真时间。`gazebo_params_file` 在默认配置之后加载，可覆盖相应参数；如果覆盖 `use_sim_time`，调用方须保持各传感器与其他节点的时间基准一致。

历史 DRL 场景原件包含用于暂停和步进物理的旧插件。通过本包入口加载时，仅在临时副本中移除该插件，保持物理连续运行，原场景及其哈希不变；历史动态障碍需由外部程序驱动。

训练与评估场景在 `/clearing/contact_collision` 报告仿真接触，过滤同一机器人的自碰撞及履带在地面或坡面的正常支承。

差速履带的轮地横向摩擦系数默认 0.25，可用仿真启动参数 `lateral_friction` 调整。

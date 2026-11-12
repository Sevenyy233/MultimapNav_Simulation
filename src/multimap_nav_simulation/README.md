# 多场景导航仿真地图

新增基础几何体大型清障机器人：
`ros2 launch multimap_nav_simulation clearing_robot_primitives.launch.py`。
默认加载 `worlds/clearing_robot/clearing_enclosed.world`（内部 50 × 50 × 20 m 封闭场景），
机械臂启动时收起并保持，履带与七轴机械臂均接入 ros2_control；五个定向 Avia 与 IMU 默认接入 FAST-LIO2。该入口不加载 Gazebo 真值里程计插件，`/odom` 和定位 TF 由 FAST-LIO2 接入桥提供。雷达射线在 Gazebo 中隐藏，五路三维点云仍正常发布。
模型、参数及接口详见 [基础几何体清障机器人](docs/clearing_robot_primitives.md)。

大型清障机器人完整导航使用本工作区的统一入口，它同时启动仿真、FAST-LIO2和原 MultiMapNav 服务器：

```bash
ros2 launch clearing_robot_plugins navigation.launch.py forced_mode:=auto
```

仿真和FAST-LIO2在线建图使用：

```bash
ros2 launch clearing_robot_plugins mapping.launch.py map_name:=clearing_enclosed
# 另一场景示例，出生位置按大型车体设置
ros2 launch clearing_robot_plugins mapping.launch.py map_name:=flat_3 spawn_x:=-6.0 spawn_y:=-7.0
```

建图时用 `/map_save` (`std_srvs/srv/Trigger`) 保存PCD；输出位置与控制命令见[清障导航及建图说明](../clearing_robot_plugins/docs/README.md)。

默认场景自动选择 OctoMap；`forced_mode` 可设为 `costmap2d`、`gridmap`、`octomap` 强制选择。
清障机器人的三模式配置和插件位于 `clearing_robot_plugins`，详见[清障导航说明](../clearing_robot_plugins/docs/README.md)和[地图模式验证](../clearing_robot_plugins/docs/map_modes_validation.md)。

ROS 2 Humble + Gazebo Classic，当前共 **17 张**：flat 5 张、terrain 5 张、floor 7 张。
场景版本：`compact_mixed_floors_dynamic_20261008_v3`。长度单位为米，world、STL 与 PCD 坐标一致。

| 地图 | 尺寸 / m | 下层 / 上层 | 动态障碍 |
| --- | --- | --- | ---: |
| flat_1 | 14.0 × 10.0 | 平地 | 0 |
| flat_2 | 26.0 × 5.6 | 平地 | 0 |
| flat_3 | 19.6 × 21.0 | 平地 | 8 |
| flat_4 | 21.0 × 16.8 | 平地 | 0 |
| flat_5 | 30.0 × 19.0 | 平地 | 0 |
| terrain_1 | 16.0 × 12.4 | 起伏地形 | 0 |
| terrain_2 | 27.0 × 7.4 | 起伏地形 | 0 |
| terrain_3 | 21.6 × 23.0 | 起伏地形 | 6 |
| terrain_4 | 28.0 × 20.7 | 起伏地形 | 0 |
| terrain_5 | 11.2 × 19.0 | 起伏地形 | 0 |
| floor_1 | 18.0 × 14.4 | 平地 / 平地 | 0 |
| floor_2 | 30.0 × 7.5 | 平地 / 平地 | 0 |
| floor_3 | 23.3 × 26.0 | 平地 / 平地 | 8 |
| floor_4 | 28.0 × 17.8 | 平地 / 平地 | 0 |
| floor_5 | 30.0 × 25.0 | 平地 / 平地 | 0 |
| floor_6 | 22.0 × 18.0 | 平地 / 起伏地形 | 0 |
| floor_7 | 24.0 × 20.0 | 起伏地形 / 起伏地形 | 0 |

floor_6 下层平坦、上层起伏，28 个静态障碍；floor_7 两层起伏，32 个静态障碍。
两张新图坡道宽 2.4 m，两端渐变并有平缓衔接区域。上下层出生点按真实地面高度配置。

flat_3、terrain_3、floor_3 分别有 8、6、8 个鲜红色动态障碍。
flat_3 与 floor_3 速度范围 0.25–0.95 m/s，terrain_3 为 0.25–0.80 m/s；不同实体速度不同。
路径分布在不同空间区域，floor_3 同时覆盖上下层。移动实体有 Gazebo 碰撞几何，激光可观测；
静态 PCD 不包含动态实体，导航使用传感器观测判断它们。

## 机器人入口

机器人模型、启动文件和参数文件按名称一一对应，`launch/` 和 `config/` 各保留四个文件：

| 机器人 | 模型 | 启动入口 | 配置 |
| --- | --- | --- | --- |
| 小型轮式差速 | `robot/robot.xacro` | `robot.launch.py` | `config/robot.yaml` |
| 小型履带 | `robot/tracked_robot.xacro` | `tracked_robot.launch.py` | `config/tracked_robot.yaml` |
| STL 清障机器人 | `robot/clearing_robot.xacro` | `clearing_robot.launch.py` | `config/clearing_robot.yaml` |
| 基础几何体清障机器人 | `robot/clearing_robot_primitives.xacro` | `clearing_robot_primitives.launch.py` | `config/clearing_robot_primitives.yaml` |

每份 YAML 包含对应机器人的控制器与节点参数。基础几何体机器人的驱动、机械臂初始位置、
行驶包络同在其 YAML 内。仿真配置只保存控制器及仿真节点参数；履带导航适配参数位于导航工程的 `bring_up/config/tracked_robot_nav_params.yaml`。
地图和出生位姿数据位于 `worlds/catalog.json`，场景设计记录位于 `worlds/scene_design.json`。
共享的零件、传感器宏、运行脚本及地图生成/验证工具保留在各自目录中。

## 构建和启动

```bash
cd /data/projects/ros2_ws/MultiMapNav_Simulation
source /opt/ros/humble/setup.bash
bash src/multimap_nav_simulation/scripts/build_livox_workspace.sh
source install/setup.bash

ros2 launch multimap_nav_simulation tracked_robot.launch.py world_file_name:=flat_1
# 双层地图在上层出生可追加 spawn_level:=upper
# 查看动态障碍：world_file_name:=flat_3、terrain_3 或 floor_3
```

`worlds/catalog.json` 统一记录地图目录、尺寸、点数、校验和、动态路径、默认出生位姿和上下层定位初值。
启动程序只设置 Gazebo 资源及插件路径、机器人模型、传感器、控制器、出生位置和可选里程计 TF。
旧地图别名 indoor_flat、terrain、two_floor 仍可使用。

四个 launch 入口不加载导航工程；基础几何体清障机器人额外提供默认启用的 FAST-LIO2 定位（`use_fast_lio:=false` 可关闭），其余入口只启动仿真。小型履带机器人需要另行接入导航时，在导航工作空间中执行：

```bash
ros2 launch bring_up tracked_robot_nav.launch.py map_name:=flat_1
```

导航与仿真的地图及出生层必须一致；导航环境和参数说明见导航工程 `src/bring_up/README.md`。两边分别启动和退出。

## 场景资源与再生成

```text
3DModel/{flat,terrain,floor}/地图名.blend  # 可编辑 Blender 场景
3DModel/{flat,terrain,floor}/地图名.stl    # 静态场景完整网格
3DModel/{flat,terrain,floor}/surfaces/     # world 使用的独立曲面网格
worlds/{flat,terrain,floor}/地图名.world
pcd/{flat,terrain,floor}/地图名.pcd
```

同名 pcd_modified 文件保持兼容，并与 pcd 完全一致。
当前场景由 Blender 脚本生成，再调用仿真包的点云采样函数，未改变论文公式。
旧 generate_map_suite.py 用于共享几何函数与旧版场景；重新生成当前场景使用：

```bash
blender --background --factory-startup --python src/multimap_nav_simulation/scripts/regenerate_diverse_suite.py -- \
  --package-dir src/multimap_nav_simulation --maps floor_6 floor_7
```

省略 --maps 将重新生成整个当前目录。局部生成保留其他地图及其设计记录。
点云按约 0.09 m 地表间距、0.08 m 墙面间距确定性采样；它是已知静态环境地图。
world_to_mesh.py 导出静态几何时会排除运动插件控制的实体。

## 验证

```bash
python3 src/multimap_nav_simulation/scripts/validate_map_suite.py
# 同时检查四个模型/启动入口/YAML 的对应关系及清障模型：
python3 src/multimap_nav_simulation/test/test_clearing_robot_primitives.py
python3 src/multimap_nav_simulation/test/test_robot_layout.py
```

检查静态 world/网格几何覆盖、PCD 数据和校验和、初始化位姿地面支撑、动态路径及颜色。
两张新图通过上下楼的离线碰撞连通性检查；这是几何可行性，实际导航和动态避障需实测。
旧实验记录对应原场景版本，不自动计入新场景验收。

## 清障机器人

```bash
ros2 launch multimap_nav_simulation clearing_robot.launch.py world_file_name:=flat_4
```

机器人模型入口为 `robot/clearing_robot.xacro`，STL 位于 `meshes/clearing_robot/`。
该机器人宽约 3.3 m、底盘长约 5 m，零位机械臂向前伸出约 7.8 m。
原 `flat_1`、`floor_1` 及部分窄通道是小型履带/差速机器人的实验场景；
清障机器人测试优先使用空间足够的 `flat_4` 或地形地图。

`stiff_ground:=true` 默认提高临时 world 副本的接触刚度，适应约 29 吨的机器人。
`arm_controller` 在启动时保持七个机械臂关节的 URDF 零位，
向 `/arm_controller/follow_joint_trajectory` 发送目标可活动机械臂。
关节布局和视觉网格保留原模型；底盘、机械臂采用简化碰撞体，适用于导航仿真。

| 接口 | 用途 |
| --- | --- |
| `/cmd_vel` | 底盘速度指令 |
| `/odom` | Gazebo 三维真值里程计 |
| `/odom_diff_drive` | 差速轮式里程计 |
| `/joint_states` | 车轮和机械臂关节 |
| `/points_raw` | 导航用 3D 点云，`lidar_link` |
| `/livox/imu_raw` | IMU，`imu_link` |

默认发布 `odom -> base_link` 真值 TF；定位系统也发布同一 TF 时，
可设置 `publish_odom_tf:=false`。

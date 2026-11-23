> 当前tracked_robot默认使用MID-360与FAST-LIO2；/odom为传感器估计，/evaluation/ground_truth仅供独立评价。外部依赖源码未在本轮修改，时间扫描适配位于本包src/sensors。最新说明见相邻MultiMapNav/src/bring_up/docs/TRACKED_LIO_NAVIGATION.md。

# 多场景导航仿真地图

清舱机器人的模型、场景和启动入口已迁入 clearing_robot_simulation 包。导航命令见 [清舱导航](../clearing_robot_plugins/README.md)。

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

每份 YAML 包含对应机器人的控制器与节点参数。仿真配置只保存控制器及仿真节点参数；履带导航适配参数位于导航工程的 `bring_up/config/tracked_robot_nav_params.yaml`。
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
# 同时检查两个模型/启动入口/YAML 的对应关系：
python3 src/multimap_nav_simulation/test/test_robot_layout.py
```

检查静态 world/网格几何覆盖、PCD 数据和校验和、初始化位姿地面支撑、动态路径及颜色。
两张新图通过上下楼的离线碰撞连通性检查；这是几何可行性，实际导航和动态避障需实测。
旧实验记录对应原场景版本，不自动计入新场景验收。

## tracked_robot：主分支传感器估计链路

当前tracked_robot默认使用自有MID-360时间扫描插件、未修改的FAST-LIO2及自有里程计/速度反馈桥接。真值仅发布到/evaluation/ground_truth，可用publish_ground_truth:=false关闭。组合启动与Release编译说明见相邻MultiMapNav仓库的src/bring_up/docs/TRACKED_LIO_NAVIGATION.md。外部依赖源码不在本轮修改范围。

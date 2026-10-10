# 多场景导航仿真地图

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

## 构建和启动

```bash
cd /data/projects/ros2_ws/MultiMapNav_Simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select multimap_nav_simulation --symlink-install
source install/setup.bash
source /data/projects/ros2_ws/MultiMapNav/install/setup.bash

ros2 launch multimap_nav_simulation gazebo_tracked_sim.launch.py world_file_name:=floor_6 navigation:=true
# floor_7 同样启动；在上层出生可追加 spawn_level:=upper
# 查看动态障碍：world_file_name:=flat_3、terrain_3 或 floor_3
```

`config/maps.yaml` 记录目录、尺寸、默认位姿、点数、文件校验和和动态路径。
`config/tracked_initial_poses.yaml` 记录上下层的定位初值及对应 world 校验和。
启动程序设置 Gazebo 资源及插件路径，并根据地图加载仿真定位参数。
旧地图别名 indoor_flat、terrain、two_floor 仍可使用。

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

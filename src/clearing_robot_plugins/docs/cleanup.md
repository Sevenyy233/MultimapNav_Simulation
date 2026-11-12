# 脚本清理

删除了下列没有外部调用引用的辅助脚本，原始备份保存在 `log/clearing_navigation_cleanup_backup/`：

- `multimap_nav_simulation/scripts/create_maps_overview.py`：旧的15图概览生成器，当前地图集已为17图，与清障机器人导航没有运行依赖。
- `multimap_nav_simulation/scripts/mesh_to_pcd.py`：通用 Open3D 转换工具，当前地图链路使用 `world_to_mesh.py` 和 `stl_to_navigation_pcd.py`，没有调用此重复入口。

其余地图生成、转换和验证脚本保留。新的地图先验生成器实际复用了 `world_to_mesh.py` 和 `stl_to_navigation_pcd.py`，现有17图工具也仍有文档和函数依赖。

截图中的四个测试文件全部有作用：机器人几何/外参检查、底盘运动回归、速度估计单元测试以及四个启动入口检查。它们用于防止本次接入破坏已有的传感器、控制与定位链路，因此保留。新增的导航和融合测试放在 `clearing_robot_plugins/test/`。

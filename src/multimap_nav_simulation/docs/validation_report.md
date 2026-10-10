# 地图验证报告

验证日期：2026-10-03。环境：ROS 2 Humble / Gazebo Classic 11。

15 张地图均已配套同名 world、完整场景 STL 和二进制 XYZ PCD，并按 flat、terrain、floor 分类安装。
原 flat_1、terrain_1、floor_1 的碰撞几何保留；terrain_1 原始 STL 的字节摘要一致，另两张地图的碰撞三角形一致。

构建通过，CTest 的 map_suite_consistency 检查通过；Python 静态检查 F/E9 通过。
静态检查覆盖文件数量与命名、资源引用、world/STL 坐标一致性、不同地图几何唯一性、PCD 头与有效数据、上下层出生点支撑面。

实际 Gazebo 检查共 27 项，全部通过：15 张地图的下层/地面检查、5 张多层地图的二楼检查、5 条坡道上楼路线，以及差速和清障机器人各一个启动入口。
每项检查确认仿真时钟前进、控制器处于 active、雷达持续发布有效点云、机器人接地及速度指令产生实际运动。

| 地图 | 测试位置/路线 | 机器人 | 实际位移 / m | 结束高度 / m | 结果 |
| --- | --- | --- | --- | --- | --- |
| flat_1 | lower | tracked | 0.208 | 0.029 | 通过 |
| flat_2 | lower | tracked | 0.208 | 0.029 | 通过 |
| flat_3 | lower | tracked | 0.208 | 0.029 | 通过 |
| flat_4 | lower | tracked | 0.208 | 0.029 | 通过 |
| flat_5 | lower | tracked | 0.208 | 0.029 | 通过 |
| terrain_1 | lower | tracked | 0.229 | -0.061 | 通过 |
| terrain_2 | lower | tracked | 0.208 | 0.029 | 通过 |
| terrain_3 | lower | tracked | 0.225 | 0.029 | 通过 |
| terrain_4 | lower | tracked | 0.225 | 0.029 | 通过 |
| terrain_5 | lower | tracked | 0.208 | 0.029 | 通过 |
| floor_1 | lower | tracked | 0.208 | 0.029 | 通过 |
| floor_1 | upper | tracked | 0.208 | 1.029 | 通过 |
| floor_2 | lower | tracked | 0.225 | 0.029 | 通过 |
| floor_2 | upper | tracked | 0.225 | 3.029 | 通过 |
| floor_3 | lower | tracked | 0.225 | 0.029 | 通过 |
| floor_3 | upper | tracked | 0.208 | 3.029 | 通过 |
| floor_4 | lower | tracked | 0.208 | 0.029 | 通过 |
| floor_4 | upper | tracked | 0.225 | 2.429 | 通过 |
| floor_5 | lower | tracked | 0.208 | 0.029 | 通过 |
| floor_5 | upper | tracked | 0.206 | 3.029 | 通过 |
| floor_2 | ramp | tracked | 11.710 | 3.029 | 通过 |
| floor_1 | ramp | tracked | 3.625 | 1.029 | 通过 |
| floor_3 | ramp | tracked | 11.712 | 3.029 | 通过 |
| floor_4 | ramp | tracked | 9.712 | 2.429 | 通过 |
| floor_5 | ramp | tracked | 11.713 | 3.029 | 通过 |
| flat_2 | lower | diff_drive | 0.188 | 0.039 | 通过 |
| flat_4 | lower | clearing | 0.242 | 0.007 | 通过 |

坡道驾驶测试在转弯平台先停车、转正，再进入下一段窄坡道。
floor_1 沿原折返坡道到达 1 m 楼层；floor_2/3/5 到达 3 m 楼层；floor_4 到达 2.4 m 楼层。
清障机器人在 flat_4 使用独立出生点 (-6, -9, 0.1, 0)，为底盘和零位机械臂预留空间。

逐项机器可读结果与日志位置保存在 validation_results.json。复现命令见上级 README.md。
测试范围为资源一致性、仿真启动、接触、传感器与控制器，以及短距离驾驶和坡道通行。

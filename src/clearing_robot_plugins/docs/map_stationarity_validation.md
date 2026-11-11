# 地图随车移动感的定位与修复

> 此记录对应本轮运动与碰撞包络修正之前的版本。最新18盒体、倒车策略和速度平顺性验收见 [运动修正验证](motion_refinement_validation.md)。

日期：2026-10-28。修改范围仅为 MultiMapNav_Simulation。

## 根因和改动

导航与建图两份RViz文件的 Global Options / Fixed Frame 已经是 `map`，但 Views / Current / Target Frame 都是 `base_link`。Orbit相机持续跟随底盘平移，即使地图数据没有变化，地图也会相对屏幕运动。

两份配置已将相机 Target Frame 改为 `map`。导航相机回到地图原点附近，保留用户已增加的OctoMap、RobotModel、雷达、路径等显示配置。修改后的配置已安装。

已打开的RViz可在 `Panels -> Views` 中把 `Target Frame` 改为 `map`，并确认 `Global Options -> Fixed Frame` 为 `map`；也可重新加载对应配置或重新打开启动入口。地图观察视角固定后，机器人会在固定的环境中移动。

## 实际运动验证

| 模式 | 实际物理位移/m | 相对运动定位误差/m | 地图检查 |
| --- | --- | --- | --- |
| 导航 | 1.8077 | 0.0149 | 先验PCD与OctoMap序列化数据、frame_id不变 |
| 建图 | 1.4305 | 0.0290 | 原有累计点云数据前缀不变，仅追加新点；map到camera_init变换不变 |

使用采集时间对应的TF将实时扫描投到地图中，扫描与静态场景的最近邻距离中位数：导航由0.1023m变为0.1029m，建图由0.1046m变为0.1053m。当前约0.1m残差包含地图和扫描采样间隔，运动前后未出现与整车位移相当的整体平移。

TF父子关系保持 `map -> odom -> base_link`；`odom -> camera_init` 和 `imu_link -> body` 为桥接别名。动态TF发布者与预期一致：导航为定位服务器、FAST-LIO桥和robot_state_publisher，建图为FAST-LIO桥和robot_state_publisher，未发现多余的动态TF发布节点。

物理状态和完整静态场景仅作为测试参考，不接入定位、建图或导航控制。建图中新出现的可见表面不一定存在于最初几帧稀疏点云中，因此不以最初点云的包围盒或质心变化作为地图漂移判断。

## 画面与记录

固定视角的导航运动前、后：

![导航运动前](/home/yangyang/ros2_ws/MultiMapNav_Simulation/log/clearing_map_motion_validation_20261028/navigation_xsoy3tga/before.png)

![导航运动后](/home/yangyang/ros2_ws/MultiMapNav_Simulation/log/clearing_map_motion_validation_20261028/navigation_xsoy3tga/after.png)

日志、参数修复前后快照、两套运动前后截图及 `summary.json` 位于 `log/clearing_map_motion_validation_20261028`。原MultiMapNav的254个源码文件哈希核对未变化。

`Five lidars`、`Current registered scan` 是当前扫描，传感器视野随机器人位置变化，点数和覆盖区域正常变化；地图应以固定墙体/箱体位置、静态体素或已累计点的坐标判断。此验证覆盖当前封闭平地、收臂、低速短程运行，不构成FAST-LIO2长期零漂移保证。

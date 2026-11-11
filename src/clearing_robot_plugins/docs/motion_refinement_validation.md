# 大型底盘运动与碰撞包络修正

> 以下为2026-10-28版本的历史验证。2026-11-03已移除目标代理及其朝向改写，当前 `/navigate_to_pose` 直接由框架 `bt_navigator` 提供，行驶方向由规划插件决定。下列历史结果不作为此次流程调整后的仿真验收结论；当前运行说明见 [导航流程](README.md#导航流程与速度平顺性)。

日期：2026-10-28。环境：ROS 2 Humble、Gazebo Classic、默认 `clearing_enclosed` 平地场景、机械臂收起。只修改 MultiMapNav_Simulation；相邻 MultiMapNav 的254个源码文件与修改前哈希一致。

## 原因与修正

1. 原铲斗单盒沿连杆坐标系倾斜，平地收臂时最低端接近地面；原小臂单盒包含较多空体积。改为底盘轴向的水平铲斗盒及四段小臂盒。铲斗名义尺寸为1.528×2.550×0.630 m，小臂名义盒体积合计减少约39%。拟合有0.025 m几何填充，另保留0.20 m硬碰撞余量。325个URDF视觉基本体的角点和内部采样都在包络内；对分段连杆另外检查其对应盒体的覆盖，不能借用无关连杆的盒体掩盖缺口。
2. 八邻域路线可能经过机器人中心可通行、长车体转向扫掠不可通行的状态；旧连接要求先旋转、再直行，空地中也可能失败。保留论文地形搜索，增加前进/倒车解析连接和SE(2)修复，正常导航不使用原地转向原语。9个默认场景开放目标通过全车路径检查；实际转向、绕行和三种地图模式另做物理验证。真正缺少全车空间、支撑层或坡度不满足的目标仍拒绝。
3. 原轨迹把路径切线和车头方向混用，更新局部路径容易重启跟随器；短时域、独立控制噪声及缺少角加加速度约束会放大左右摆动。现在分别处理车体朝向与有符号行驶方向，线段投影跟踪，保留可复用路径时间戳，加入时序相关噪声、转向反转代价、角加速度/加加速度和驱动力预算。规划半径3.5 m，控制半径基准2.5 m；局部预留0.35 m额外跟踪空间。
4. 默认目标策略为 `prefer_reverse`。后方目标优先保留当前车头方向，倒车速度为负；可用 `goal_heading_mode:=requested` 严格采用RViz目标箭头。目标适配通过独立公共/内部动作端点接入原MultiMapNav导航器，RViz显示最终采用的车头目标。
5. 规划与恢复指令统一经过末级速度整形器，再进入带S形限制的履带驱动。目标变化、普通停车和原始指令超时均保持有限加速度、有限加加速度。真实碰撞紧急停车及仿真时钟回退允许立即归零。传感器中断期间保留空间路径、由健康门禁阻止运动，恢复后重新验证路径，避免单点占位路径让跟随器停留在旧暂停状态。

## 实际仿真结果

| 场景 | 结果 |
| --- | --- |
| 前进到(2,0)，再倒车到(-2,0)，再转弯到(2,2) | 三个目标成功；后方目标位置误差0.0276 m，保持车头朝向；最终位置误差0.0565 m、朝向误差0.0321 rad |
| 未知箱体绕行，第1轮 | 成功；397次独立全车检查未相交；终点位置误差0.0842 m、定位误差0.0307 m |
| 未知箱体绕行，第2轮 | 成功；459次独立全车检查未相交；终点位置误差0.1551 m、定位误差0.0285 m |
| 强制Costmap2D，直行6 m | 成功；终点位置误差0.1634 m、定位误差0.0203 m |
| 强制GridMap，直行6 m | 成功；终点位置误差0.1199 m、定位误差0.0157 m |
| 行驶中暂停感知5个仿真秒 | 实际车速降至0.00398 m/s；数据恢复后继续完成导航，终点位置误差0.2094 m |
| 后退/旋转恢复 | 实际后退0.5657 m；旋转0.2494 rad，均成功 |
| 独立建图入口 | 里程计位移1.4822 m，地图由2605点增长并保存28574点；未加载先验及导航服务器 |

默认auto选择OctoMap；三种模式的地图话题及全部8个原框架生命周期节点均检查。导航栈只有一个 `/cmd_vel` 发布者，公开动作由目标适配节点提供，原导航器只提供内部动作端点。Gazebo物理状态仅作为测试参考，没有用于定位或控制。

未知箱体为先验中不存在的1×1×1.5 m箱体，中心(5.5,5.5,0.75)，目标(10,6,0)。开始绕行前必须收到箱体回波；沿实际物理位姿，用独立分离轴检测所有18个名义盒体与箱体相交情况，共856次全车检查。规划和在线安全检查另外保留硬余量。两轮分别使用build10和build11；后者只修正健康中断时的路径交接，健康状态下的规划控制相同。不得用本次成功替代任意环境安全保证。

连续换向场景按带仿真时间戳的 `/clearing/cmd_vel_smooth` 测得最大线/角加速度0.1800 m/s²、0.0800 rad/s²，最大线/角加加速度0.4000 m/s³、0.1500 rad/s³。履带应用指令的最大线加速度0.1796 m/s²、线加加速度0.5965 m/s³，低于驱动的0.3/0.6限制。轨迹中超过0.025 rad/s的转向符号反转次数，95分位为0、最大为1。此指标描述本组轨迹，不能等同于任何目标下都没有转向变化。

## 回归与复现

- 37项Python检查通过：URDF/外参/布局、收臂确认、地图入口与出生边界、全部基本体包络、目标朝向策略及驱动换向S形曲线。
- 12项C++检查通过：地图体积、三模式语义、坡度侧倾、论文搜索、完整扫掠、9个开放目标、倒车车体朝向、终点越过修正及耦合动力约束。
- 五雷达原始融合契约，建图无先验融合契约，运动时自体过滤/真实障碍保留契约均通过；几何与感知实现自对应测试后未变。
- 实际导航与建图RViz主窗口均已检查：水平铲斗、分段小臂、18盒共享模型、固定地图视角；保留导航RobotModel和OctoMap显示。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR="$PWD/log/unit_ros" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/clearing_robot_plugins/test/test_*.py \
  src/multimap_nav_simulation/test/test_arm_confirmation.py \
  src/multimap_nav_simulation/test/test_clearing_robot_primitives.py \
  src/multimap_nav_simulation/test/test_robot_layout.py
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 ctest --test-dir build/clearing_robot_plugins --output-on-failure
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase chassis
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase obstacle
python3 src/clearing_robot_plugins/test/smoke_navigation.py --phase sensor_loss
```

完整证据位于 `log/clearing_motion_refinement_20261028`，`summary.json` 指向最终验收场景及源码/二进制哈希。各场景保存 `result.json`、`debug.json`、`launch.log`；早期探索失败保留用于排障，未列为最终通过结果。终止自己启动的仿真时产生的SIGINT/exit -2属于清理日志。

本次为平地、收臂、低速仿真验收。质量与惯量来自URDF，驱动力预算采用保守系数；履带土壤接触、实机牵引/滑移、坡地动态及机械臂运动未做辨识。二维强制模式仍可能因垂直投影保守拒绝某些通道；地图匹配用于约束漂移，不能承诺长期零漂移。

![导航实际RViz](/data/projects/ros2_ws/MultiMapNav_Simulation/log/clearing_motion_refinement_20261028/clearing_navigation_0yx9zs54/rviz.png)

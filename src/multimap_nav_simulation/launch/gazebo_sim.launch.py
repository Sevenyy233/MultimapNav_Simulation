#!/usr/bin/env python3
# ================================================================
# gazebo_sim.launch.py — Gazebo 仿真启动文件
#
# 启动内容:
#   1. Gazebo 服务器 + 客户端（加载 world）
#   2. robot_state_publisher（发布 /tf）
#   3. spawn 机器人模型到 Gazebo
#   4. Gazebo 内部 ros2_control + diff_drive_controller + joint_state_broadcaster
#
# 传感器话题（默认建图配置）:
#   /livox/lidar  — Avia 固态激光雷达 (livox_ros_driver2/CustomMsg)
#   /livox/imu    — IMU 数据
#
# 里程计话题:
#   /odom           — gazebo_ros_p3d 地面真值里程计（libgazebo_ros_p3d.so，机器人模型内加载）
#   /odom_diff_drive— 轮式里程计（diff_drive_controller 发布，仅作参考，不再占用 /odom）
#
# TF 树结构:
#   lidar ─FAST-LIO(可选)─> base_link ─URDF─> avia / imu_link / 轮子
#
# 说明:
#   - odom_tf_broadcaster（odom→base_link 真值 TF）已从本文件移出，
#     由 octomap_mapping.launch.py 按需启动，避免与 FAST-LIO 发布的
#     lidar→base_link 形成 base_link 双父 TF 冲突（tf2 会拒绝后插入的边）。
#   - FAST-LIO 建图模式: 先启动本文件，再启动 fastlio2_avia.launch.py。
#   - 真值建图模式:     先启动本文件，再启动 octomap_mapping.launch.py。
#
# GAZEBO_PLUGIN_PATH:
#   必须包含工作区 install/ros2_livox_simulation/lib，否则 Gazebo 找不到
#   libros2_livox.so，/livox/lidar 不会有数据（Avia 雷达插件无法加载）。
#
# 控制话题:
#   /cmd_vel  — 差速驱动速度指令
#
# 用法:
#   ros2 launch multimap_nav_simulation gazebo_sim.launch.py
#   ros2 launch multimap_nav_simulation gazebo_sim.launch.py world:=worlds/cabin.world
#   ros2 launch multimap_nav_simulation gazebo_sim.launch.py headless:=true
# ================================================================
import re
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ==================================================================
    # 路径
    # ==================================================================
    pkg_share = get_package_share_directory('multimap_nav_simulation')
    default_world = os.path.join(pkg_share, 'worlds', 'room.world')
    xacro_file = os.path.join(pkg_share, 'robot', 'robot.xacro')

    # ==================================================================
    # GAZEBO_PLUGIN_PATH: 追加工作区 Livox 仿真插件目录（libros2_livox.so）
    # ==================================================================
    livox_sim_prefix = get_package_prefix('ros2_livox_simulation')
    livox_plugin_dir = os.path.join(livox_sim_prefix, 'lib')
    existing_gazebo_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    gazebo_plugin_path = os.pathsep.join(
        [p for p in [existing_gazebo_path, livox_plugin_dir] if p]
    )
    set_gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH', gazebo_plugin_path
    )
    # ==================================================================
    # 参数声明
    # ==================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file   = LaunchConfiguration('world', default=default_world)
    use_gui      = LaunchConfiguration('gui', default='true')
    headless     = LaunchConfiguration('headless', default='false')

    # ==================================================================
    # robot_description (xacro → URDF)
    # ==================================================================
    robot_desc_value = xacro.process_file(xacro_file).toxml()
    robot_desc_value = re.sub(r'<!--.*?-->', '', robot_desc_value, flags=re.DOTALL)
    robot_desc_value = ' '.join(robot_desc_value.split())

    robot_description = {'robot_description': robot_desc_value}

    # ==================================================================
    # Gazebo 服务器
    # ==================================================================
    gzserver_cmd = [
        'gzserver', '--verbose',
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        world_file,
    ]

    gzserver = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=gzserver_cmd,
        output='screen',
    )

    gzserver_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=gzserver_cmd,
        output='screen',
    )

    # ==================================================================
    # Gazebo 客户端（GUI）
    # ==================================================================
    gzclient = ExecuteProcess(
        condition=IfCondition(use_gui),
        cmd=['gzclient'],
        output='screen',
    )

    # ==================================================================
    # robot_state_publisher
    # ==================================================================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ==================================================================
    # odom_tf_broadcaster 已移至 octomap_mapping.launch.py
    # （FAST-LIO 建图模式下 base_link TF 由 FAST-LIO 提供，
    #   避免 odom→base_link 与 lidar→base_link 双父冲突）
    # ==================================================================

    # ==================================================================
    # spawn 机器人
    # ==================================================================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'diff_drive_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.15',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # ==================================================================
    # 控制器 spawner
    # 机器人生成后，gazebo_ros2_control 会在 Gazebo 内部创建 controller_manager。
    # 因此这里不再手动启动 ros2_control_node，只在实体生成完成后再请求加载控制器。
    # ==================================================================
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_spawner, diff_drive_spawner],
                )
            ],
        )
    )


    # ==================================================================
    # 返回 LaunchDescription
    # ==================================================================
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),

        set_gazebo_plugin_path,

        gzserver,
        gzserver_headless,
        gzclient,

        robot_state_pub,
        spawn_entity,
        spawn_controllers,
    ])

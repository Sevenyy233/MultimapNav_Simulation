#!/usr/bin/env python3
# ================================================================
# gazebo_tracked_sim.launch.py — 履带机器人 Gazebo 仿真启动文件
#
# 启动内容:
#   1. Gazebo 服务器 + 客户端（加载 world）
#   2. robot_state_publisher（发布 /tf）
#   3. spawn tracked_robot 到 Gazebo
#   4. Gazebo 内部 ros2_control + diff_drive_controller + joint_state_broadcaster
#
# 传感器话题（与 MultiMapNav / robot.xacro 对齐）:
#   /points_raw              — 3D 多线激光雷达（frame=lidar_link）
#   /livox/imu_raw           — Gazebo IMU（frame=imu_link）
#   /depth_camera/depth/points
#
# 里程计话题:
#   /odom            — gazebo_ros_p3d 地面真值（libgazebo_ros_p3d.so）
#   /odom_diff_drive — 轮式开环里程计（diff_drive_controller）
#
# TF 树结构:
#   odom ─odom_tf_broadcaster─> base_link ─robot_state_publisher─> lidar_link / imu_link / 轮子
#
# 控制话题:
#   /cmd_vel  — 差速驱动速度指令（同侧 4 轮同步）
#
# 用法:
#   ros2 launch multimap_nav_simulation gazebo_tracked_sim.launch.py
#   ros2 launch multimap_nav_simulation gazebo_tracked_sim.launch.py world_file_name:=two_floor
#   ros2 launch multimap_nav_simulation gazebo_tracked_sim.launch.py headless:=true
# ================================================================
import re
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_share = get_package_share_directory('multimap_nav_simulation')
    xacro_file = os.path.join(pkg_share, 'robot', 'tracked_robot.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = LaunchConfiguration('world_file_name')
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        [world_file_name, '.world'],
    ])
    use_gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    robot_desc_value = xacro.process_file(xacro_file).toxml()
    robot_desc_value = re.sub(r'<!--.*?-->', '', robot_desc_value, flags=re.DOTALL)
    robot_desc_value = ' '.join(robot_desc_value.split())

    robot_description = {'robot_description': robot_desc_value}

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

    gzclient = ExecuteProcess(
        condition=IfCondition(use_gui),
        cmd=['gzclient'],
        output='screen',
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'tracked_robot',
            '-topic', 'robot_description',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

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

    odom_tf_broadcaster = Node(
        package='multimap_nav_simulation',
        executable='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world_file_name',
            default_value='cafe',
            description=(
                'World file basename in the worlds directory '
                '(cafe, indoor_flat, terrain, two_floor)'
            ),
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        # 轮子底面相对 base_link 为 -0.03 m；0.05 m 仅保留 2 cm 落下高度，
        # 避免默认出生时过大接触冲击。
        DeclareLaunchArgument('spawn_z', default_value='0.05'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),

        gzserver,
        gzserver_headless,
        gzclient,

        robot_state_pub,
        spawn_entity,
        spawn_controllers,
        odom_tf_broadcaster,
    ])

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
# 传感器话题（内置仿真传感器）:
#   /livox/lidar_PointCloud2 — 3D 多线激光雷达（lidar_3d，Gazebo ray 传感器）
#   /livox/imu_raw           — Gazebo IMU (sensor_msgs/Imu, m/s²)
#
# 里程计话题:
#   /odom           — gazebo_ros_p3d 地面真值里程计（libgazebo_ros_p3d.so，机器人模型内加载）
#   /odom_diff_drive— 轮式里程计（diff_drive_controller 发布，仅作参考，不再占用 /odom）
#
# TF 树结构:
#   odom ─odom_tf_broadcaster─> base_link ─robot_state_publisher─> lidar_link / imu_link / 轮子
#
# 说明:
#   - 本文件只启动仿真，不发布 odom→base_link（由需要真值 TF 的上层启动文件按需创建）。
#
# 控制话题:
#   /cmd_vel  — 差速驱动速度指令
#
# 用法:
#   ros2 launch multimap_nav_simulation gazebo_sim.launch.py
#   ros2 launch multimap_nav_simulation gazebo_sim.launch.py world_file_name:=room
#   ros2 launch multimap_nav_simulation gazebo_sim.launch.py headless:=true
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

    # ==================================================================
    # 路径
    # ==================================================================
    pkg_share = get_package_share_directory('multimap_nav_simulation')
    xacro_file = os.path.join(pkg_share, 'robot', 'robot.xacro')

    # ==================================================================
    # 参数声明
    # ==================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = LaunchConfiguration('world_file_name')
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        [world_file_name, '.world'],
    ])
    use_gui      = LaunchConfiguration('gui', default='true')
    headless     = LaunchConfiguration('headless', default='false')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

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
    # odom_tf_broadcaster 由需要真值 TF 的上层启动文件按需创建；本文件
    # 不创建动态 TF 发布者，避免多个启动入口重复发布 odom→base_link。
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
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-Y', spawn_yaw,
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

    odom_tf_broadcaster = Node(
        package='multimap_nav_simulation',
        executable='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            #'parent_frame_id': 'odom',
            # 真值位姿使用独立子帧，不与 FAST-LIO 抢占 base_link。
            #'child_frame_id': 'ground_truth_base_link',
        }]
    )


    # ==================================================================
    # 返回 LaunchDescription
    # ==================================================================
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world_file_name',
            default_value='cafe',
            description=(
                'World file basename in the worlds directory '
                '(cabin, cafe, outdoor, room, terrain, uneven_terrain)'
            ),
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
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

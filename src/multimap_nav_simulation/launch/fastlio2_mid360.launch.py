#!/usr/bin/env python3
# ================================================================
# fastlio2_mid360.launch.py — FAST-LIO2 Mid-360 建图启动文件
#
# 启动内容:
#   1. FAST-LIO2 lio_node（加载 mid360_lio.yaml）
#   2. 静态 TF 桥接 odom → lidar（FAST-LIO 的 world 帧与控制器里程计帧对齐）
#   3. RViz2 可视化（复用 fastlio2_avia.rviz，话题与坐标系一致）
#
# TF 树结构:
#   odom ─静态恒等─> lidar ─FAST-LIO 动态─> base_link ─URDF─> mid360 / imu_link / 轮子
#
# 前提:
#   先启动 Gazebo 仿真: ros2 launch multimap_nav_simulation gazebo_sim.launch.py
#   确认话题有数据: ros2 topic hz /livox/mid360, ros2 topic hz /livox/imu
#
# 用法:
#   ros2 launch multimap_nav_simulation fastlio2_mid360.launch.py
# ================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('multimap_nav_simulation')
    config_path = os.path.join(pkg_share, 'config', 'mid360_lio.yaml')

    lio_node = Node(
        package='fastlio2',
        namespace='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{'config_path': config_path, 'use_sim_time': True}],
    )

    # 静态 TF 桥接: odom → lidar（恒等）
    # diff_drive_controller 的世界帧是 odom，FAST-LIO 的世界帧是 lidar，
    # 两者初始重合。没有这条桥接，odom 与 lidar 两棵 TF 子树互相不可达。
    odom_lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_lidar_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar'],
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        namespace='fastlio2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'fastlio2_avia.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([lio_node, odom_lidar_static_tf, rviz_node])

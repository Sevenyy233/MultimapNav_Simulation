#!/usr/bin/env python3
# ================================================================
# fastlio2_avia.launch.py — FAST-LIO2 Avia 建图启动文件
#
# 启动内容:
#   1. FAST-LIO2 lio_node（加载 avia_lio.yaml）
#   2. 静态 TF 桥接 odom → lidar（锚定到 Avia 雷达初始位姿）
#   3. RViz2 可视化
#
# TF 树结构:
#   odom ─静态(雷达初始位姿)─> lidar ─FAST-LIO 动态─> base_link ─URDF─> avia / imu_link / 轮子
#
# 前提:
#   先启动 Gazebo 仿真: ros2 launch multimap_nav_simulation gazebo_sim.launch.py
#   确认话题有数据: ros2 topic hz /livox/lidar, ros2 topic hz /livox/imu
#
# 用法:
#   ros2 launch multimap_nav_simulation fastlio2_avia.launch.py
#
# 说明:
#   - 不要同时启动 octomap_mapping.launch.py（其 odom_tf_broadcaster 会与
#     FAST-LIO 的 lidar→base_link 形成 base_link 双父 TF 冲突）。
#   - 静态 TF 把 FAST-LIO 的 world（lidar 系，原点在雷达初始位姿）锚定到
#     odom 系：spawn z=0.15 + avia 相对 base_link (0.84, 0, 0.57) 且 rpy=(0, 0.35, 0)
#     → xyz="0.84 0 0.72" rpy="0 0.35 0"。
# ================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('multimap_nav_simulation')
    config_path = os.path.join(pkg_share, 'config', 'avia_lio.yaml')

    # 根分区只读时（`/` 挂载为 ro），ROS 默认日志目录 ~/.ros/log 不可写，
    # spdlog 打不开日志文件会抛异常导致节点 SIGABRT（exit code -6）。
    # 把 ROS 日志目录重定向到工作区内可写目录，避免该问题。
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    os.environ.setdefault('ROS_LOG_DIR', os.path.join(ws_root, '.ros_log'))

    lio_node = Node(
        package='fastlio2',
        namespace='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{'config_path': config_path, 'use_sim_time': True}],
    )

    # 静态 TF 桥接: odom → lidar（锚定到 Avia 雷达初始位姿）
    # diff_drive_controller 的世界帧是 odom，FAST-LIO 的世界帧是 lidar
    # （原点固定在雷达初始位姿）。把 lidar 锚定到 odom 系中的雷达初始位姿
    # （spawn z=0.15 + avia xyz="0.84 0 0.57" rpy="0 0.35 0"）后，
    # FAST-LIO 估计的 base_link 位姿可与世界系对齐。
    odom_lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_lidar_static_tf',
        arguments=['0.84', '0', '0.72', '0', '0.35', '0', 'odom', 'lidar'],
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

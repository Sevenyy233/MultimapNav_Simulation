#!/usr/bin/env python3
# ================================================================
# fastlio2_avia_pgo.launch.py — FAST-LIO2 + PGO 全局优化建图启动文件
#
# 启动内容:
#   1. FAST-LIO2 lio_node（加载 avia_lio.yaml，局部里程计 + 局部地图）
#   2. PGO pgo_node（加载 pgo_avia.yaml，位姿图优化 + 回环检测）
#   3. RViz2 可视化（fixed frame = map）
#
# TF 树结构:
#   map ─pgo 优化─> lidar ─fastlio2 动态─> base_link ─URDF─> avia / imu_link / 轮子
#
# 说明:
#   - PGO 发布 map→lidar（优化后的全局位姿），FAST-LIO2 发布 lidar→base_link，
#     因此本模式不再需要静态 TF odom→lidar（避免 lidar 双父 TF 冲突）。
#   - 不要同时启动 octomap_mapping.launch.py / fastlio2_avia.launch.py。
#   - 机器人需要移动（发布 /cmd_vel）才会产生关键帧与回环，建图更精确。
#   - 保存优化后地图（目录需预先存在）:
#       mkdir -p /tmp/pgo_map
#       ros2 service call /pgo/save_maps interface/srv/SaveMaps \
#           "{file_path: '/tmp/pgo_map', save_patches: false}"
#     输出: /tmp/pgo_map/map.pcd（全局优化地图）
#
# 前提:
#   先启动 Gazebo 仿真: ros2 launch multimap_nav_simulation gazebo_sim.launch.py
#   确认话题有数据: ros2 topic hz /livox/lidar, ros2 topic hz /livox/imu
#
# 用法:
#   ros2 launch multimap_nav_simulation fastlio2_avia_pgo.launch.py
# ================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('multimap_nav_simulation')
    lio_config_path = os.path.join(pkg_share, 'config', 'avia_lio.yaml')
    pgo_config_path = os.path.join(pkg_share, 'config', 'pgo_avia.yaml')
    pgo_rviz_path = os.path.join(
        get_package_share_directory('pgo'), 'rviz', 'pgo.rviz')

    # ------------------------------------------------------------------
    # FAST-LIO2 局部里程计 + 局部地图
    # ------------------------------------------------------------------
    lio_node = Node(
        package='fastlio2',
        namespace='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{'config_path': lio_config_path, 'use_sim_time': True}],
    )

    # ------------------------------------------------------------------
    # PGO 位姿图优化（回环检测 + 全局优化，发布 map→lidar）
    # ------------------------------------------------------------------
    pgo_node = Node(
        package='pgo',
        namespace='pgo',
        executable='pgo_node',
        name='pgo_node',
        output='screen',
        parameters=[{'config_path': pgo_config_path, 'use_sim_time': True}],
    )

    # ------------------------------------------------------------------
    # RViz2（fixed frame = map，显示全局地图 / 轨迹 / 回环）
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        namespace='pgo',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', pgo_rviz_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([lio_node, pgo_node, rviz_node])

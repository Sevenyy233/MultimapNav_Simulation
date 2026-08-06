#!/usr/bin/env python3
# ================================================================
# octomap_mapping.launch.py — octomap_server 3D 建图启动文件
#
# 使用工作区源码包 octomap_server（src/octomap_mapping/octomap_server，
# 即 GitHub OctoMap/octomap_mapping v2.3.1），由本地 colcon 构建出的
# octomap_server_node 进行建图；不再使用 /opt/ros/humble 的系统安装包。
#
# 默认输入话题: /livox/lidar_PointCloud2 (sensor_msgs/PointCloud2)
#               —— gazebo_sim.launch.py 默认启用前装 Avia 仿真雷达
# 说明:
#   Avia 安装在 base_link 前方 x=1.40m、上方 z=0.95m，并向下倾斜约 20°，
#   且水平视场约 70°，因此 RViz 中的 octomap 会主要出现在机器人前半部。
#   这是当前传感器布局的正常表现，不是 odom/base_link/avia 之间的 TF 错位。
# 固定坐标系: odom（gazebo_sim.launch.py 中的 odom_tf_broadcaster
#           由 gazebo_ros_p3d 地面真值发布 odom→base_link，无需 FAST-LIO）
#
# 用法:
#   1. ros2 launch multimap_nav_simulation gazebo_sim.launch.py
#   2. ros2 launch multimap_nav_simulation octomap_mapping.launch.py
#   （不要同时启动 fastlio2_avia，避免 base_link 双父 TF 冲突）
#
# 本文件同时启动 odom_tf_broadcaster：
#   订阅 /odom（gazebo_ros_p3d 地面真值里程计），发布 odom→base_link TF。
#   该真值 TF 原属于 gazebo_sim.launch.py，为消除与 FAST-LIO 发布的
#   lidar→base_link 之间的 base_link 双父冲突，已移至本建图启动文件按需启动。
#
#   保存地图:
#   ros2 launch multimap_nav_simulation save_octomap.launch.py \
#       octomap_path:=/tmp/map.ot
#
#   换用其他点云话题（例如 Mid-360 或 FAST-LIO 的 world_cloud）:
#   ros2 launch multimap_nav_simulation octomap_mapping.launch.py \
#       cloud_in:=/livox/mid360_PointCloud2
#
#   验证当前加载的是本地包（而不是系统包）:
#     source install/setup.bash
#     ros2 pkg prefix octomap_server
#     # 应输出 .../MultiMapNav_Simulation/install/octomap_server
# ================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('multimap_nav_simulation')
    default_params = os.path.join(pkg_share, 'config', 'octomap_mapping.yaml')

    params_file = LaunchConfiguration('params_file', default=default_params)
    cloud_in = LaunchConfiguration('cloud_in', default='/livox/lidar_PointCloud2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    point_cloud_max_z = LaunchConfiguration('point_cloud_max_z', default='1.8')
    occupancy_max_z = LaunchConfiguration('occupancy_max_z', default='1.8')

    # ----------------------------------------------------------------
    # odom_tf_broadcaster
    # 订阅 /odom（gazebo_ros_p3d 地面真值），发布 odom → base_link TF，
    # 供 octomap_server 等节点使用（本文件是唯一需要真值 TF 的建图模式）。
    # ----------------------------------------------------------------
    odom_tf_broadcaster = Node(
        package='multimap_nav_simulation',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ----------------------------------------------------------------
    # octomap_server 建图节点（工作区源码包 src/octomap_mapping/octomap_server）
    # ----------------------------------------------------------------
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'point_cloud_max_z': point_cloud_max_z,
            'occupancy_max_z': occupancy_max_z,
        }],
        remappings=[('cloud_in', cloud_in)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='octomap_server 参数文件路径',
        ),
        DeclareLaunchArgument(
            'cloud_in',
            default_value='/livox/lidar_PointCloud2',
            description='输入点云话题 (sensor_msgs/PointCloud2)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用 Gazebo 仿真时间',
        ),
        DeclareLaunchArgument(
            'point_cloud_max_z',
            default_value='1.8',
            description='点云预处理保留的最大 z，高于该值的点不进入 OctoMap',
        ),
        DeclareLaunchArgument(
            'occupancy_max_z',
            default_value='1.8',
            description='最终发布地图保留的最大 z，用于裁掉高处占据体素',
        ),
        octomap_server_node,
        odom_tf_broadcaster,
    ])

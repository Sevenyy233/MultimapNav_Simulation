#!/usr/bin/env python3
# ================================================================
# save_octomap.launch.py — 保存 octomap_server 已建好的地图
#
# 通过 octomap_saver_node 调用 octomap_server 的 octomap_full /
# octomap_binary 服务，一次性取回当前地图并写入文件，随后自动退出。
# 使用的包: 工作区源码包 src/octomap_mapping/octomap_server（与建图节点同包）。
#
# 用法:
#   ros2 launch multimap_nav_simulation save_octomap.launch.py \
#       octomap_path:=/tmp/map.ot
#   ros2 launch multimap_nav_simulation save_octomap.launch.py \
#       octomap_path:=/tmp/map.bt full:=false
#
# 说明:
#   - .ot = 完整文本地图（可读、体积大）
#   - .bt = 二进制压缩地图（体积小、加载快）
#   - full:=true 请求完整地图，false 请求压缩地图（默认 true）
#   - 保存前请先启动 octomap_mapping.launch.py
# ================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    octomap_path = LaunchConfiguration('octomap_path', default='/home/yangyang/ros2_ws/MultiMapNav_Simulation/map.ot')
    full = LaunchConfiguration('full', default='true')

    octomap_saver_node = Node(
        package='octomap_server',
        executable='octomap_saver_node',
        name='octomap_saver',
        output='screen',
        parameters=[{'octomap_path': octomap_path, 'full': full}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'octomap_path',
            default_value='/home/yangyang/ros2_ws/MultiMapNav_Simulation/map.ot',
            description='保存路径，支持 .bt（二进制）或 .ot（文本）',
        ),
        DeclareLaunchArgument(
            'full',
            default_value='true',
            description='true 请求完整地图(octomap_full)，false 请求压缩地图(octomap_binary)',
        ),
        octomap_saver_node,
    ])

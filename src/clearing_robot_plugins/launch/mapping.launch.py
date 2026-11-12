"""Clearing robot simulation and online FAST-LIO2 mapping, without a prior map."""
from datetime import datetime
from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from multimap_nav_simulation.map_catalog import resolve_world
from clearing_robot_plugins.mapping_scene import validate_catalog_spawn


def setup(context):
    value = lambda key: context.launch_configurations[key]
    package = Path(get_package_share_directory('clearing_robot_plugins'))
    simulation = Path(get_package_share_directory('multimap_nav_simulation'))
    name = value('world_file').strip() or value('map_name').strip()
    if name == 'clearing_enclosed':
        name = 'clearing_robot/clearing_enclosed'
    world = resolve_world(simulation, name)
    pose = [float(value(key)) for key in ('spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw')]
    if not all(math.isfinite(v) for v in pose):
        raise RuntimeError('Spawn coordinates must be finite numbers.')
    validate_catalog_spawn(simulation, world, pose, package/'config/robot_model.yaml')
    if value('map_output_file').strip():
        output = Path(value('map_output_file')).expanduser().absolute()
    else:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        output = Path.cwd()/'maps'/f'{world.stem}_{stamp}'/'map.pcd'
    if output.suffix.lower() != '.pcd':
        raise RuntimeError('map_output_file must have a .pcd extension.')
    output.parent.mkdir(parents=True, exist_ok=True)
    arguments = {key: value(key) for key in (
        'headless', 'gui', 'spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw', 'lidar_downsample')}
    arguments.update({
        'world_file': str(world), 'use_fast_lio': 'true',
        'primary_lidar_topic': '/clearing/lidar_fused',
        'primary_lidar_frame': 'imu_link', 'fused_lidar_in_imu': 'true',
        'fast_lio_config_file': str(package/'config/mapping.yaml'),
        'fast_lio_map_file': str(output),
    })
    actions = [
        LogInfo(msg=f'Mapping world: {world}'),
        LogInfo(msg=f'PCD output: {output}; save while running with '
                    'ros2 service call /map_save std_srvs/srv/Trigger "{}"'),
        Node(package='clearing_robot_plugins', executable='clearing_perception',
             parameters=[{'use_sim_time': True, 'mapping_only': True,
                          'robot_model_file': str(package/'config/robot_model.yaml')}],
             output='screen'),
        # FAST-LIO starts through the simulation's existing arm-ready chain.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            simulation/'launch/clearing_robot_primitives.launch.py')),
            launch_arguments=arguments.items()),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='mapping_map_to_odom', parameters=[{'use_sim_time': True}],
             arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0',
                        '--pitch', '0', '--roll', '0',
                        '--frame-id', 'map', '--child-frame-id', 'odom']),
    ]
    if value('use_rviz').lower() in ('true', '1', 'yes'):
        actions.append(Node(package='rviz2', executable='rviz2',
                            parameters=[{'use_sim_time': True}],
                            arguments=['-d', str(package/'rviz/mapping.rviz')], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='clearing_enclosed',
                              description='Gazebo scene: clearing_enclosed, flat_1..5, '
                                          'terrain_1..5, floor_1..7; adjust spawn for the large body.'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='Optional absolute .world path overriding map_name.'),
        DeclareLaunchArgument('map_output_file', default_value='',
                              description='PCD output; default ./maps/<scene_timestamp>/map.pcd.'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('lidar_downsample', default_value='8'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.03'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])

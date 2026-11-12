#!/usr/bin/env python3
"""Simulation only: primitive clearing robot with its arm held before enabling drive."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import xacro

from multimap_nav_simulation.map_catalog import gazebo_environment, resolve_world


def _after_success(action, next_actions):
    def on_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'Primitive clearing robot startup failed (exit {event.returncode}).'))]
        return next_actions
    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit))


def _launch_setup(context):
    share = Path(get_package_share_directory('multimap_nav_simulation'))
    value = lambda name: context.launch_configurations[name]
    boolean = lambda name: value(name).lower() in ('true', '1', 'yes')
    world_name = value('world_file').strip() or value('world_file_name')
    world = resolve_world(share, world_name)
    config_file = share / 'config' / 'clearing_robot_primitives.yaml'
    downsample = int(value('lidar_downsample'))
    if not 1 <= downsample <= 24000:
        raise RuntimeError('lidar_downsample must be between 1 and 24000.')
    document = xacro.process_file(
        str(share / 'robot' / 'clearing_robot_primitives.xacro'),
        mappings={'lateral_friction': value('lateral_friction')})
    for plugin in document.getElementsByTagName('plugin'):
        if plugin.getAttribute('filename') != 'libros2_livox.so':
            continue
        sensor = plugin.parentNode
        # Keep sensing active without drawing Gazebo's trigger-scan rays.
        for visualize in sensor.getElementsByTagName('visualize'):
            visualize.firstChild.data = 'false'
        # Vendor URDF only configures the plugin's rays. Enable the Gazebo
        # trigger sensor in headless mode too, without editing that URDF.
        always_on = document.createElement('always_on')
        always_on.appendChild(document.createTextNode('true'))
        sensor.appendChild(always_on)
        # These regular-grid rays only trigger the plugin, which traces its
        # own CSV rays. Keep this duplicate grid small for five CPU sensors.
        ray = plugin.getElementsByTagName('ray')[0]
        for axis in ('horizontal', 'vertical'):
            ray.getElementsByTagName(axis)[0].getElementsByTagName('samples')[0].firstChild.data = '2'
        plugin.getElementsByTagName('downsample')[0].firstChild.data = str(downsample)
        sensor.appendChild(ray.cloneNode(deep=True))
    description = document.toxml()
    description = ' '.join(re.sub(r'<!--.*?-->', '', description, flags=re.DOTALL).split())
    sim_time = boolean('use_sim_time')
    use_fast_lio = boolean('use_fast_lio')
    environment = gazebo_environment(share)
    environment['GAZEBO_PLUGIN_PATH'] = (
        str(Path(get_package_prefix('ros2_livox_simulation')) / 'lib')
        + ':' + environment['GAZEBO_PLUGIN_PATH'])
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description, 'use_sim_time': sim_time}],
        output='screen')
    server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', str(world)],
        additional_env=environment, output='screen')
    wait_for_world = Node(
        package='multimap_nav_simulation', executable='wait_for_gazebo_clock.py',
        parameters=[{'timeout': 120.0}], output='screen')
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'clearing_robot_primitives', '-topic', 'robot_description',
                   '-x', value('spawn_x'), '-y', value('spawn_y'), '-z', value('spawn_z'),
                   '-Y', value('spawn_yaw'), '-timeout', '120'], output='screen')

    def spawner(controller):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[controller, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '120'], output='screen')

    joint_states = spawner('joint_state_broadcaster')
    arm = spawner('arm_controller')
    fold = Node(
        package='multimap_nav_simulation', executable='initialize_clearing_arm.py',
        parameters=[str(config_file), {'use_sim_time': sim_time}], output='screen')
    tracks = spawner('tracks_velocity_controller')
    drive = Node(
        package='multimap_nav_simulation', executable='clearing_tracks_drive.py',
        parameters=[str(config_file),
                    {'use_sim_time': sim_time}], output='screen')
    localization = []
    if use_fast_lio:
        estimator_parameters = {'use_sim_time': sim_time}
        if boolean('fused_lidar_in_imu'):
            estimator_parameters.update({'mapping.extrinsic_T': [0.0, 0.0, 0.0],
                'mapping.extrinsic_R': [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],
                'mapping.fov_degree': 360.0, 'point_filter_num': 5})
        estimator_files = [str(config_file)]
        if value('fast_lio_config_file'):
            overrides = Path(value('fast_lio_config_file')).expanduser()
            if not overrides.is_file():
                raise RuntimeError(f'FAST-LIO configuration does not exist: {overrides}')
            estimator_files.append(str(overrides))
        if value('fast_lio_map_file'):
            estimator_parameters['map_file_path'] = value('fast_lio_map_file')
        localization = [
            Node(package='multimap_nav_simulation', executable='fast_lio_sim_bridge',
                 parameters=[str(config_file), {'use_sim_time': sim_time,
                             'lidar_topic': value('primary_lidar_topic'),
                             'lidar_frame': value('primary_lidar_frame')}], output='screen'),
            Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
                 parameters=[*estimator_files, estimator_parameters],
                 remappings=[('/Odometry', '/fast_lio/odometry'),
                             ('/tf', '/fast_lio/tf')], output='screen'),
        ]

    actions = [
        RegisterEventHandler(OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited.'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=drive,
            on_exit=[EmitEvent(event=Shutdown(reason='Track command bridge exited.'))])),
        _after_success(wait_for_world, [spawn]),
        _after_success(spawn, [joint_states]),
        _after_success(joint_states, [arm]),
        _after_success(arm, [fold]),
        _after_success(fold, [tracks]),
        _after_success(tracks, [drive, *localization, LogInfo(
            msg='Primitive clearing robot ready: arm held in low carry pose; /cmd_vel enabled.')]),
        LogInfo(msg=f'Primitive clearing robot world: {world}'),
        state_publisher, server, wait_for_world,
    ]
    if boolean('gui') and not boolean('headless'):
        actions.append(ExecuteProcess(cmd=['gzclient'], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_file_name', default_value='clearing_robot/clearing_enclosed',
                              description='World below worlds/; default is the 50 x 50 x 20 m enclosure.'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='Optional absolute .world path overriding world_file_name.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.03',
                              description='Ground projection height; 3 cm initial clearance on a flat floor.'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('lateral_friction', default_value='0.03',
                              description='ODE lateral friction for the approximate skid-steer tracks.'),
        DeclareLaunchArgument('lidar_downsample', default_value='8',
                              description='Avia CSV stride: 8 gives 3000 rays/sensor/frame; 1 traces all 24000.'),
        DeclareLaunchArgument('use_fast_lio', default_value='true',
                              description='Localize using front-left Avia + IMU; publish /odom and odom -> base_link.'),
        DeclareLaunchArgument('fast_lio_config_file', default_value='',
                              description='Optional FAST-LIO YAML overrides (e.g. mapping output).'),
        DeclareLaunchArgument('fast_lio_map_file', default_value='',
                              description='Optional PCD output path for FAST-LIO map_save.'),
        DeclareLaunchArgument('primary_lidar_topic', default_value='/livox/lidar_3',
                              description='Primary CustomMsg input; navigation supplies the body-filtered topic.'),
        DeclareLaunchArgument('primary_lidar_frame', default_value='rslidar_3'),
        DeclareLaunchArgument('fused_lidar_in_imu', default_value='false',
                              description='Five-lidar fusion already expressed in IMU coordinates; use identity extrinsic.'),
        OpaqueFunction(function=_launch_setup),
    ])

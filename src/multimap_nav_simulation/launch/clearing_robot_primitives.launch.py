#!/usr/bin/env python3
"""Hold the primitive clearing robot in a low carry pose before enabling drive."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
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
    description = xacro.process_file(
        str(share / 'robot' / 'clearing_robot_primitives.xacro'),
        mappings={'lateral_friction': value('lateral_friction')}).toxml()
    description = ' '.join(re.sub(r'<!--.*?-->', '', description, flags=re.DOTALL).split())
    sim_time = boolean('use_sim_time')
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description, 'use_sim_time': sim_time}],
        output='screen')
    server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', str(world)],
        additional_env=gazebo_environment(share), output='screen')
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
        _after_success(tracks, [drive, LogInfo(
            msg='Primitive clearing robot ready: arm held in low carry pose; /cmd_vel enabled.')]),
        LogInfo(msg=f'Primitive clearing robot world: {world}'),
        state_publisher, server, wait_for_world,
    ]
    if boolean('gui') and not boolean('headless'):
        actions.append(ExecuteProcess(cmd=['gzclient'], output='screen'))
    if boolean('publish_odom_tf'):
        actions.append(Node(
            package='multimap_nav_simulation', executable='odom_tf_broadcaster',
            parameters=[{'use_sim_time': sim_time, 'odom_topic': '/odom',
                         'output_odom_topic': '/truth/odometry',
                         'source_frame_id': 'world', 'parent_frame_id': 'odom'}],
            output='screen'))
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
        DeclareLaunchArgument('publish_odom_tf', default_value='true',
                              description='Publish truth odom -> base_link TF and /truth/odometry.'),
        OpaqueFunction(function=_launch_setup),
    ])

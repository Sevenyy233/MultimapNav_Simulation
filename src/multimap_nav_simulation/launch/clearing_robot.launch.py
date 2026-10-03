#!/usr/bin/env python3
"""Launch the clearing robot in Gazebo at the original URDF zero pose."""

import os
from pathlib import Path
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import xacro

# Flat ground for the 5 m x 3.3 m chassis, facing the map interior; spawning
# at the terrain origin embeds the robot in a mound and it falls through.
WORLD_SPAWN_POSES = {
    'cabin': ('0.0', '0.0', '0.05', '0.0'),
    'terrain': ('10.0', '-4.0', '0.3', '3.14159'),
    'terrain_with_wall': ('3.0', '9.0', '0.3', '3.14159'),
}
DEFAULT_SPAWN_POSE = ('0.0', '0.0', '0.05', '0.0')
GROUND_KP = '1e9'
GROUND_KD = '1e5'


def _after_success(action, next_actions):
    """Stop startup on failure instead of enabling a partially initialized robot."""
    def on_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'Clearing robot startup failed (exit code {event.returncode}).'))]
        return next_actions

    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit))


def _launch_setup(context):
    pkg_share = Path(get_package_share_directory('multimap_nav_simulation'))
    value = lambda name: context.launch_configurations[name]
    boolean = lambda name: value(name).lower() in ('true', '1', 'yes')

    world_name = value('world_file').strip() or value('world_file_name').strip()
    world = Path(os.path.expanduser(world_name))
    if not world.is_absolute():
        world = pkg_share / 'worlds' / world
    if world.suffix != '.world':
        world = world.with_suffix('.world')
    if not world.is_file():
        raise RuntimeError(f'World file does not exist: {world}')
    world_spawn = WORLD_SPAWN_POSES.get(world.stem, DEFAULT_SPAWN_POSE)
    spawn_pose = [value(f'spawn_{axis}').strip() or default
                  for axis, default in zip(('x', 'y', 'z', 'yaw'), world_spawn)]
    if boolean('stiff_ground'):
        # Gazebo combines contact kp in series, so a world's soft ground (kp 1e6,
        # tuned for the small robot) caps the stiffness: this ~29 t robot sinks
        # about 10 cm and falls through the mesh once it drives.
        text = world.read_text()
        stiff = re.sub(r'<kp>[^<]*</kp>', f'<kp>{GROUND_KP}</kp>', text)
        stiff = re.sub(r'<kd>[^<]*</kd>', f'<kd>{GROUND_KD}</kd>', stiff)
        if stiff != text:
            world = Path(tempfile.gettempdir()) / f'clearing_robot_{world.name}'
            world.write_text(stiff)

    description = xacro.process_file(str(pkg_share / 'robot' / 'clearing_robot.xacro')).toxml()
    # Gazebo Classic passes this XML through the ROS argument parser. Comments
    # with colons/newlines can be interpreted as YAML parameter syntax.
    description = re.sub(r'<!--.*?-->', '', description, flags=re.DOTALL)
    description = ' '.join(description.split())
    sim_time = boolean('use_sim_time')
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', str(world)],
        additional_env={
            'GAZEBO_MODEL_PATH': os.pathsep.join(filter(None, [
                str(pkg_share / 'models'), os.environ.get('GAZEBO_MODEL_PATH', '')])),
        },
        output='screen',
    )
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description, 'use_sim_time': sim_time}],
        output='screen',
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'clearing_robot', '-topic', 'robot_description',
                   '-x', spawn_pose[0], '-y', spawn_pose[1],
                   '-z', spawn_pose[2], '-Y', spawn_pose[3], '-timeout', '120'],
        output='screen',
    )
    wait_for_world = Node(
        package='multimap_nav_simulation', executable='wait_for_gazebo_clock.py',
        parameters=[{'timeout': 120.0}], output='screen')

    def spawner(controller):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[controller, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '120'],
            output='screen',
        )

    joint_states = spawner('joint_state_broadcaster')
    arm = spawner('arm_controller')
    hold_arm = Node(
        package='multimap_nav_simulation', executable='initialize_clearing_arm.py',
        parameters=[{'use_sim_time': sim_time, 'positions': [0.0] * 7,
                     'move_duration': 2.0, 'timeout': 120.0}],
        output='screen',
    )
    drive = spawner('diff_drive_controller')

    actions = [
        RegisterEventHandler(OnProcessExit(
            target_action=gzserver,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited.'))])),
        _after_success(wait_for_world, [spawn]),
        _after_success(spawn, [joint_states]),
        _after_success(joint_states, [arm]),
        _after_success(arm, [hold_arm]),
        _after_success(hold_arm, [drive]),
        _after_success(drive, [LogInfo(msg='Clearing robot ready; /cmd_vel enabled.')]),
        LogInfo(msg=f'World: {world}; spawn x y z yaw: {spawn_pose}'),
        state_publisher,
        gzserver,
        wait_for_world,
    ]
    if boolean('gui') and not boolean('headless'):
        actions.append(ExecuteProcess(cmd=['gzclient'], output='screen'))
    if boolean('publish_odom_tf'):
        actions.append(Node(
            package='multimap_nav_simulation', executable='odom_tf_broadcaster',
            parameters=[{'use_sim_time': sim_time}], output='screen'))
    return actions


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('world_file_name', default_value='cabin',
                              description='World basename in this package, e.g. cabin or terrain.'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='Optional absolute .world path; overrides world_file_name.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Suppress gzclient even when gui is true.'),
        DeclareLaunchArgument('stiff_ground', default_value='true',
                              description='Load a temporary world copy whose contact kp/kd are raised '
                                          'for this heavy robot; the original world file is unchanged.'),
    ]
    arguments.extend(DeclareLaunchArgument(
        f'spawn_{axis}', default_value='',
        description=f'Spawn {axis}; empty uses the per-world default in WORLD_SPAWN_POSES.')
        for axis in ('x', 'y', 'z', 'yaw'))
    arguments += [
        DeclareLaunchArgument('publish_odom_tf', default_value='true',
                              description='Publish ground truth odom -> base_link TF.'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])

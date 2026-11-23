#!/usr/bin/env python3
"""@file
@brief 启动清障机器人、控制器和可选的 FAST-LIO 定位。
"""

from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from multimap_nav_simulation.map_catalog import gazebo_environment, resolve_world


def _after_success(action, next_actions):
    """@brief 前序进程成功退出后启动后续动作；失败时关闭仿真。"""
    def on_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'Clearing robot startup failed (exit {event.returncode}).'))]
        return next_actions
    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit))


def _launch_setup(context):
    """@brief 解析启动参数并按顺序装载机器人和控制器。"""
    share = Path(get_package_share_directory('clearing_robot_simulation'))

    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        return value(name).lower() in ('true', '1', 'yes')

    world_name = value('world_file').strip() or value('world_file_name')
    world = resolve_world(share, world_name)
    # 历史 DRL 场景保留原始文件及哈希；启动时仅从临时副本移除步进插件。
    # 该插件会暂停物理，不能用于当前持续运行的仿真。
    world_tree = ET.parse(world)
    world_node = world_tree.getroot().find('world')
    steppers = [plugin for plugin in world_node.findall('plugin')
                if plugin.get('filename') == 'libclearing_simulation_stepper.so']
    temporary_world = None
    if steppers:
        for plugin in steppers:
            world_node.remove(plugin)
        temporary_world = tempfile.TemporaryDirectory(prefix='clearing-world-')
        world = Path(temporary_world.name) / world.name
        world_tree.write(world, encoding='utf-8', xml_declaration=True)
    config_file = share / 'config' / 'clearing_robot.yaml'
    downsample = int(value('lidar_downsample'))
    if not 1 <= downsample <= 24000:
        raise RuntimeError('lidar_downsample must be between 1 and 24000.')
    document = xacro.process_file(
        str(share / 'robot' / 'clearing_robot.xacro'),
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
        + ':' + str(Path(get_package_prefix('clearing_robot_simulation')) / 'lib')
        + ':' + environment['GAZEBO_PLUGIN_PATH'])
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description, 'use_sim_time': sim_time,
                     'publish_frequency': 100.0}],
        output='screen')
    # Gazebo 传感器插件自身也是 ROS 节点；确保 Avia 消息使用仿真时钟。
    gazebo_arguments = ['--ros-args', '--params-file', str(share / 'config' / 'gazebo_sim_time.yaml')]
    if value('gazebo_params_file'):
        gazebo_arguments += ['--params-file', value('gazebo_params_file')]
    gazebo_arguments += ['--']
    server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', str(world), *gazebo_arguments],
        additional_env=environment, output='screen')
    wait_for_world = Node(
        package='clearing_robot_simulation', executable='wait_for_gazebo_clock.py',
        parameters=[{'timeout': 120.0}], output='screen')
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'clearing_robot', '-topic', 'robot_description',
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
        package='clearing_robot_simulation', executable='initialize_clearing_arm.py',
        parameters=[str(config_file), {'use_sim_time': sim_time}], output='screen')
    tracks = spawner('tracks_velocity_controller')
    drive = Node(
        package='clearing_robot_simulation', executable='clearing_tracks_drive.py',
        remappings=[
            ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('/tracks_velocity_controller/commands', LaunchConfiguration('track_commands_topic')),
            ('/clearing/cmd_vel_applied', LaunchConfiguration('cmd_vel_applied_topic')),
        ],
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
            msg='Clearing robot ready: arm held in low carry pose; /cmd_vel enabled.')]),
        LogInfo(msg=f'Clearing robot world: {world}'),
        state_publisher, server, wait_for_world,
    ]
    if temporary_world is not None:
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: temporary_world.cleanup())))
        actions.append(LogInfo(msg='历史场景的步进插件已从临时副本移除，物理持续运行。'))
    if boolean('gui') and not boolean('headless'):
        actions.append(ExecuteProcess(
            cmd=['gzclient'], additional_env=environment, output='screen'))
    return actions


def generate_launch_description():
    """@brief 声明仿真和传感器配置参数。"""
    return LaunchDescription([
        DeclareLaunchArgument('world_file_name', default_value='training',
                              description='World below worlds/; default is the training scene.'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='Optional absolute .world path overriding world_file_name.'),
        DeclareLaunchArgument('gazebo_params_file', default_value='',
                              description='Optional Gazebo ROS parameters file, including clock publication rate.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.03',
                              description='Ground projection height; 3 cm initial clearance on a flat floor.'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('lateral_friction', default_value='0.25',
                              description='ODE lateral friction for the approximate skid-steer tracks.'),
        DeclareLaunchArgument('lidar_downsample', default_value='8',
                              description='Avia CSV stride: 8 gives 3000 rays/sensor/frame; 1 traces all 24000.'),
        DeclareLaunchArgument('use_fast_lio', default_value='false',
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
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Command input for the legacy track bridge.'),
        DeclareLaunchArgument('track_commands_topic',
                              default_value='/tracks_velocity_controller/commands',
                              description='Wheel command output for the legacy track bridge.'),
        DeclareLaunchArgument('cmd_vel_applied_topic', default_value='/clearing/cmd_vel_applied',
                              description='Applied-command feedback from the legacy track bridge.'),
        OpaqueFunction(function=_launch_setup),
    ])

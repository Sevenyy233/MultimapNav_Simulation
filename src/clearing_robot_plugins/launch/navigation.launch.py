"""The large clearing robot's own MultiMapNav deployment, without tracked profiles."""
import json
import hashlib
import math
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from clearing_robot_plugins.configuration import expand_profiles, node_parameters, validate_forced_mode


def setup(context):
    value = lambda key: context.launch_configurations[key]
    boolean = lambda key: value(key).lower() in ('true', '1', 'yes')
    package = Path(get_package_share_directory('clearing_robot_plugins'))
    simulation = Path(get_package_share_directory('multimap_nav_simulation'))
    forced_mode = validate_forced_mode(value('forced_mode'))
    robot_configurations, mode_configurations = expand_profiles(
        yaml.safe_load((package/'config/navigation.yaml').read_text()),
        yaml.safe_load((package/'config/modes.yaml').read_text()),
        package/'behavior_trees/navigation.xml')
    def parameters(name):
        result = node_parameters(robot_configurations, name)
        result.update(node_parameters(mode_configurations, name))
        return result
    map_file = Path(value('map_file') or package/'maps/clearing_enclosed.pcd')
    support = Path(value('support_file') or package/'maps/clearing_support.pcd')
    model = package/'config/robot_model.yaml'
    for path in (map_file, support, model):
        if not path.is_file():
            raise RuntimeError(f'Missing clearing navigation asset: {path}')
    world = value('world_file')
    if bool(value('map_file')) != bool(value('support_file')):
        raise RuntimeError('map_file and support_file must be supplied together.')
    if world and (not value('map_file') or not value('support_file')):
        raise RuntimeError('A custom world requires matching map_file and support_file priors.')
    provenance = json.loads((package/'maps/provenance.json').read_text())
    def verify(path, expected):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Stale clearing navigation geometry/map: {path}. '
                               'Regenerate assets with scripts/prepare_assets.py before navigation.')
    for item in provenance['geometry_inputs']:
        verify(Path(get_package_share_directory(item['package']))/item['path'], item['sha256'])
    verify(model, provenance['model_asset_sha256'])
    robot_config = yaml.safe_load((simulation/'config/clearing_robot_primitives.yaml').read_text())
    drive = robot_config['clearing_tracks_drive']['ros__parameters']
    if (robot_config['initialize_clearing_arm']['ros__parameters']['positions'] != provenance['stowed_joints']
            or any(drive[key] != provenance[key] for key in ('wheel_radius', 'wheel_separation'))):
        raise RuntimeError('Stowed arm / track dimensions changed; regenerate the collision model.')
    if not value('map_file') and not value('support_file'):
        verify(simulation/'worlds/clearing_robot/clearing_enclosed.world', provenance['world_sha256'])
        verify(map_file, provenance['map_asset_sha256'])
        verify(support, provenance['support_asset_sha256'])
    initial = json.loads(value('initial_pose'))
    if len(initial) != 7 or not all(math.isfinite(float(x)) for x in initial):
        raise RuntimeError('initial_pose must be [x,y,z,qx,qy,qz,qw].')
    norm = math.sqrt(sum(float(x)**2 for x in initial[3:]))
    if norm < 1e-9:
        raise RuntimeError('initial_pose has an invalid quaternion.')
    initial = [float(x) for x in initial[:3]] + [float(x)/norm for x in initial[3:]]
    common = {'use_sim_time': True, 'scene.map_file': str(map_file),
              'scene.support_file': str(support), 'scene.robot_model_file': str(model),
              'scene.map_source': 'topic', 'scene.map_mode': 'auto'}
    actions = [Node(package='clearing_robot_plugins', executable='clearing_velocity_governor',
                    parameters=[parameters('clearing_velocity_governor'), common], output='screen')]

    if boolean('start_simulation'):
        arguments = {'headless': value('headless'), 'use_fast_lio': 'true',
                     'primary_lidar_topic': '/clearing/lidar_fused',
                     'primary_lidar_frame': 'imu_link', 'fused_lidar_in_imu': 'true',
                     'spawn_x': str(initial[0]), 'spawn_y': str(initial[1]),
                     'spawn_z': str(initial[2] + 0.02),
                     'spawn_yaw': str(math.atan2(2*(initial[6]*initial[5]+initial[3]*initial[4]),
                                                1-2*(initial[4]**2+initial[5]**2)))}
        if world:
            arguments['world_file'] = world
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            simulation/'launch/clearing_robot_primitives.launch.py')), launch_arguments=arguments.items()))
    actions.append(Node(package='clearing_robot_plugins', executable='clearing_perception',
        output='screen', parameters=[parameters('clearing_perception'), {'use_sim_time': True, 'map_file': str(map_file),
                                    'support_file': str(support), 'robot_model_file': str(model)}]))
    actions.append(Node(package='multimap_nav_servers', executable='navigation_mode_selector_server',
        name='navigation_mode_selector_server', output='screen',
        parameters=[parameters('navigation_mode_selector_server'), {
            'pcd_file_path': str(map_file), 'forced_mode': forced_mode, 'use_sim_time': True}]))
    for name in ('map_server', 'localization_server', 'global_planner_server',
                 'local_planner_server', 'trajectory_planner_server', 'recovery_server'):
        extra = {'map_file_path': str(map_file)} if name == 'map_server' else {}
        if name == 'localization_server':
            extra['initial_pose'] = initial
        actions.append(Node(package='multimap_nav_servers', executable=name, name=name,
                            output='screen', parameters=[parameters(name), common, extra],
                            remappings=[('cmd_vel','/clearing/cmd_vel_raw')]))
    actions.append(Node(package='multimap_nav_bt', executable='bt_navigator', name='bt_navigator',
                        remappings=[('cmd_vel','/clearing/cmd_vel_raw')],
                        output='screen', parameters=[parameters('bt_navigator'), common, {
                            'default_bt_xml_filename': str(package/'behavior_trees/navigation.xml')}]))
    actions.append(Node(package='multimap_nav_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager', output='screen',
                        parameters=[parameters('lifecycle_manager'), {'use_sim_time': True}]))
    if boolean('use_rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', str(package/'rviz/navigation.rviz')],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_simulation', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('forced_mode', default_value='auto',
                              description='auto/costmap2d/gridmap/octomap; selected once during bringup'),
        DeclareLaunchArgument('world_file', default_value=''),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('support_file', default_value=''),
        DeclareLaunchArgument('initial_pose', default_value='[0,0,0.01,0,0,0,1]'),
        OpaqueFunction(function=setup),
    ])

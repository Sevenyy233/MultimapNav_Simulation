"""Start assisted navigation after gazebo_tracked_sim reports robot ready."""
import hashlib
import json
import math
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from multimap_nav_simulation.map_catalog import load_catalog, resolve_world, spawn_pose


def setup(context):
    package=Path(get_package_share_directory('multimap_nav_simulation'))
    config=context.launch_configurations
    world=resolve_world(package,config['world_file_name'])
    record=load_catalog(package)[world.stem]
    pcd=package/record['pcd']
    if not pcd.is_file():raise FileNotFoundError(f'PCD is unavailable: {pcd}')
    x,y,z,yaw=map(float,spawn_pose(package,world,config['spawn_level'],'tracked'))
    surveyed=yaml.safe_load((package/'config/tracked_initial_poses.yaml').read_text())[world.stem]
    if hashlib.sha256(world.read_bytes()).hexdigest()!=surveyed['world_sha256']:
        raise ValueError('World changed since the documented initial map pose was validated.')
    initial=surveyed[config['spawn_level']]
    if abs(x-initial['x'])>1e-6 or abs(y-initial['y'])>1e-6:
        raise ValueError('Catalog spawn moved since the initial map pose was validated.')
    parameters=yaml.safe_load((package/'config/gazebo_assisted_localization_params.yaml').read_text())
    parameters['localization_server']['ros__parameters']['initial_pose']=[
        x,y,float(initial['base_z']),0.0,0.0,math.sin(yaw/2),math.cos(yaw/2)]
    if config['initial_pose'].strip():
        initial_pose=[float(v) for v in json.loads(config['initial_pose'])]
        if len(initial_pose)!=7 or not all(math.isfinite(v) for v in initial_pose) or sum(v*v for v in initial_pose[3:])<1e-12:
            raise ValueError('initial_pose must be finite [x,y,z,qx,qy,qz,qw] with a nonzero quaternion')
        parameters['localization_server']['ros__parameters']['initial_pose']=initial_pose
    temporary=tempfile.TemporaryDirectory(prefix='multimapnav_assisted_')
    path=Path(temporary.name)/'navigation.yaml'
    path.write_text(yaml.safe_dump(parameters))
    def cleanup(event,context):
        temporary.cleanup()
        return []
    navigation=Path(get_package_share_directory('bring_up'))/'launch/auto_nav.launch.py'
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation)),launch_arguments={
            'pcd_file':str(pcd),'extra_params':str(path),'start_odom_bridge':'false',
            'forced_mode':config['forced_mode'],'use_rviz':config['use_rviz']}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_file_name',default_value='flat_1'),
        DeclareLaunchArgument('spawn_level',default_value='lower',description='Match the simulation spawn level.'),
        DeclareLaunchArgument('forced_mode',default_value='auto'),
        DeclareLaunchArgument('initial_pose',default_value='',description='Optional explicit map pose for a custom simulation start.'),
        DeclareLaunchArgument('use_rviz',default_value='true'),
        OpaqueFunction(function=setup)])

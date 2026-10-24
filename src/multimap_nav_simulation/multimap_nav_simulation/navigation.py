"""Map-matched assisted navigation used by the tracked robot launch entry."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

from .map_catalog import load_catalog, load_initial_poses, spawn_pose


NAVIGATION_NODES = ('localization_server', 'trajectory_planner_server', 'bt_navigator',
                    'local_planner_server', 'lifecycle_manager')


def navigation_actions(package, world, level, initial_pose='', forced_mode='auto', use_rviz='true'):
    """Validate the map/pose pair and start the external navigation stack."""
    record = load_catalog(package)[world.stem]
    pcd = package / record['pcd']
    if not pcd.is_file():
        raise FileNotFoundError(f'PCD is unavailable: {pcd}')
    x, y, z, yaw = map(float, spawn_pose(package, world, level, 'tracked'))
    surveyed = load_initial_poses(package)[world.stem]
    if hashlib.sha256(world.read_bytes()).hexdigest() != surveyed['world_sha256']:
        raise ValueError('World changed since its documented initial pose was validated.')
    initial = surveyed[level]
    if abs(x-initial['x']) > 1e-6 or abs(y-initial['y']) > 1e-6:
        raise ValueError('Catalog spawn changed since its initial pose was validated.')
    profile = yaml.safe_load((package / 'config/tracked_robot.yaml').read_text())
    parameters = {name: deepcopy(profile[name]) for name in NAVIGATION_NODES}
    position = [x, y, float(initial['base_z']), 0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]
    if initial_pose.strip():
        position = [float(value) for value in json.loads(initial_pose)]
        if (len(position) != 7 or not all(math.isfinite(value) for value in position)
                or sum(value*value for value in position[3:]) < 1e-12):
            raise ValueError('initial_pose must be finite [x,y,z,qx,qy,qz,qw] with a nonzero quaternion')
    parameters['localization_server']['ros__parameters']['initial_pose'] = position
    temporary = tempfile.TemporaryDirectory(prefix='multimapnav_assisted_')
    path = Path(temporary.name) / 'navigation.yaml'
    path.write_text(yaml.safe_dump(parameters))

    def cleanup(event, context):
        temporary.cleanup()
        return []

    navigation = Path(get_package_share_directory('bring_up')) / 'launch/auto_nav.launch.py'
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation)), launch_arguments={
            'pcd_file': str(pcd), 'extra_params': str(path), 'start_odom_bridge': 'false',
            'forced_mode': forced_mode, 'use_rviz': use_rviz}.items()),
    ]

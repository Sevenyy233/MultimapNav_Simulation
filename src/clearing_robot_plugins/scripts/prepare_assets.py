#!/usr/bin/env python3
"""Export a reproducible prior map and the shared Marker/collision OBB model.

The prior comes from static world geometry, never from robot truth odometry.
Run after sourcing the simulation workspace; outputs are package source assets.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml

# The source form can regenerate assets before installing the new fitter.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from clearing_robot_plugins.collision_geometry import primitive_corners, fit_boxes


def matrix(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(
            origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return t


def pcd(path, points):
    points = np.asarray(points, dtype='<f4')
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ('# Clearing robot static geometry\nVERSION 0.7\nFIELDS x y z\n'
              f'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\n'
              f'HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    with path.open('wb') as stream:
        stream.write(header.encode()); stream.write(points.tobytes())


def unique(points, leaf):
    _, indices = np.unique(np.floor(points/leaf).astype(np.int32), axis=0, return_index=True)
    return points[np.sort(indices)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--world', type=Path)
    parser.add_argument('--spacing', type=float, default=.18)
    args = parser.parse_args()
    simulation = Path(get_package_share_directory('multimap_nav_simulation'))
    world = args.world or simulation/'worlds/clearing_robot/clearing_enclosed.world'
    config = yaml.safe_load((simulation/'config/clearing_robot_primitives.yaml').read_text())
    angles = config['initialize_clearing_arm']['ros__parameters']['positions']
    robot = ET.fromstring(xacro.process_file(str(simulation/'robot/clearing_robot_primitives.xacro')).toxml())
    transforms = {'base_link': np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        remaining = []
        for joint in pending:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                remaining.append(joint); continue
            t = matrix(joint.find('origin'))
            if joint.get('name') in [f'joint{i}' for i in range(1, 8)]:
                i = int(joint.get('name')[5:])-1
                rot = np.eye(4); rot[:3, :3] = Rotation.from_rotvec(
                    np.fromstring(joint.find('axis').get('xyz'), sep=' ')*angles[i]).as_matrix()
                t = t@rot
            transforms[joint.find('child').get('link')] = transforms[parent]@t
        if len(remaining) == len(pending):
            raise RuntimeError('Disconnected robot model')
        pending = remaining
    selected = ['base_link', 'left_track_belt', 'right_track_belt']
    selected += [f'link{i}' for i in range(1, 8)] + [f'rslidar_{i}' for i in range(1, 6)]
    boxes = []
    coverage = {}
    for name in selected:
        primitives = []
        for visual in robot.findall(f"link[@name='{name}']/visual"):
            t = transforms[name]@matrix(visual.find('origin'))
            geometry = visual.find('geometry')
            shape = geometry.find('box')
            if shape is not None:
                half = np.fromstring(shape.get('size'), sep=' ')/2
            else:
                shape = geometry.find('cylinder')
                if shape is None:
                    raise RuntimeError(f'Unsupported model primitive: {name}')
                half = np.array([float(shape.get('radius'))]*2+[float(shape.get('length'))/2])
            primitives.append(primitive_corners(t, half))
        fitted = fit_boxes(name, primitives, transforms[name][:3, :3])
        boxes.extend(fitted)
        coverage[name] = len(primitives)
    # Stowed rigid-body inertia about the total centre of mass; useful for
    # conservative planar force/yaw-moment limits, not a soil traction model.
    bodies = []
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        t = transforms[link.get('name')]@matrix(inertial.find('origin'))
        mass = float(inertial.find('mass').get('value'))
        v = {k: float(x) for k, x in inertial.find('inertia').attrib.items()}
        inertia = np.array([[v['ixx'],v['ixy'],v['ixz']], [v['ixy'],v['iyy'],v['iyz']],
                            [v['ixz'],v['iyz'],v['izz']]])
        bodies.append((mass,t,t[:3,:3]@inertia@t[:3,:3].T))
    mass = sum(v[0] for v in bodies)
    com = sum(m*t[:3,3] for m,t,_ in bodies)/mass
    inertia = np.zeros((3,3))
    for m,t,i in bodies:
        r=t[:3,3]-com
        inertia += i+m*(np.dot(r,r)*np.eye(3)-np.outer(r,r))
    dynamics = {'mass_kg': mass, 'yaw_inertia_kg_m2': float(inertia[2,2]),
                'center_of_mass': com.tolist()}
    (args.output/'config').mkdir(parents=True, exist_ok=True)
    (args.output/'config/robot_model.yaml').write_text(yaml.safe_dump(
        {'stowed_joints': angles, 'dynamics': dynamics, 'boxes': boxes}, sort_keys=False))

    sys.path.insert(0, str(simulation/'scripts'))
    spec = importlib.util.spec_from_file_location('clearing_world_export', simulation/'scripts/world_to_mesh.py')
    exporter = importlib.util.module_from_spec(spec); spec.loader.exec_module(exporter)
    triangles = exporter.world_triangles(world, simulation)
    rng = np.random.default_rng(20261027)
    clouds, supports = [], []
    for triangle in triangles:
        cross = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
        norm = np.linalg.norm(cross)
        if norm < 1e-9:
            continue
        count = max(3, int(np.ceil(norm/args.spacing**2)))
        uv = rng.random((count, 2)); uv[uv.sum(axis=1)>1] = 1-uv[uv.sum(axis=1)>1]
        samples = triangle[0] + uv[:, :1]*(triangle[1]-triangle[0]) + uv[:, 1:]*(triangle[2]-triangle[0])
        clouds.append(samples)
        if cross[2]/norm > .7:
            valid = (samples[:, 2] > -.05) & (samples[:, 2] < 10)
            if valid.any(): supports.append(samples[valid])
    cloud = unique(np.concatenate(clouds), args.spacing*.75)
    support = unique(np.concatenate(supports), args.spacing*.75)
    pcd(args.output/'maps/clearing_enclosed.pcd', cloud)
    pcd(args.output/'maps/clearing_support.pcd', support)
    geometry_inputs = []
    for package, name in (
        ('multimap_nav_simulation', 'robot/clearing_robot_primitives.xacro'),
        ('multimap_nav_simulation', 'robot/part/clearing_arm_geometry.xacro'),
        ('multimap_nav_simulation', 'robot/sensors/imu.xacro'),
        ('ros2_livox_simulation', 'urdf/avia.xacro')):
        path = Path(get_package_share_directory(package))/name
        geometry_inputs.append({'package': package, 'path': name,
                                'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    drive = config['clearing_tracks_drive']['ros__parameters']
    (args.output/'maps/provenance.json').write_text(json.dumps({
        'world': str(world), 'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
        'model_sha256': hashlib.sha256((simulation/'robot/clearing_robot_primitives.xacro').read_bytes()).hexdigest(),
        'sampling_spacing_m': args.spacing, 'map_points': len(cloud), 'support_points': len(support),
        'collision_boxes': len(boxes), 'primitive_counts': coverage, 'stowed_dynamics': dynamics,
        'truth_odometry_used': False,
        'geometry_inputs': geometry_inputs, 'stowed_joints': angles,
        'wheel_radius': drive['wheel_radius'], 'wheel_separation': drive['wheel_separation'],
        'model_asset_sha256': hashlib.sha256((args.output/'config/robot_model.yaml').read_bytes()).hexdigest(),
        'map_asset_sha256': hashlib.sha256((args.output/'maps/clearing_enclosed.pcd').read_bytes()).hexdigest(),
        'support_asset_sha256': hashlib.sha256((args.output/'maps/clearing_support.pcd').read_bytes()).hexdigest()}, indent=2)+'\n')
    print(f'Exported {len(boxes)} shared collision/Marker boxes and {len(cloud)} static map points')


if __name__ == '__main__':
    main()

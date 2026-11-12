"""Check regenerated envelopes against independent URDF primitive volumes."""
import importlib.util
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml

ROOT = Path(__file__).parents[1]
SIM = ROOT.parent/'multimap_nav_simulation'


def test_all_stowed_primitive_volumes_are_covered():
    robot = ET.fromstring(xacro.process_file(str(SIM/'robot/clearing_robot_primitives.xacro')).toxml())
    model = yaml.safe_load((ROOT/'config/robot_model.yaml').read_text())
    angles = dict(zip((f'joint{i}' for i in range(1, 8)), model['stowed_joints']))

    def transform(origin):
        t = np.eye(4)
        if origin is not None:
            t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        return t

    poses = {'base_link': np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        next_pending = []
        for joint in pending:
            parent = joint.find('parent').get('link')
            if parent not in poses:
                next_pending.append(joint)
                continue
            rotation = np.eye(4)
            if joint.get('name') in angles:
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                rotation[:3, :3] = Rotation.from_rotvec(axis*angles[joint.get('name')]).as_matrix()
            poses[joint.find('child').get('link')] = poses[parent]@transform(joint.find('origin'))@rotation
        assert len(next_pending) < len(pending)
        pending = next_pending
    rng = np.random.default_rng(20261028)
    samples = np.concatenate([np.array(list(product([-1., 1.], repeat=3))), rng.uniform(-1, 1, (100, 3))])
    source_names = {b.get('source_link', b['name']) for b in model['boxes']}
    for b in model['boxes']:
        if b['name'].startswith('rslidar_') or 'track_belt' in b['name']:
            assert np.allclose(Rotation.from_quat(b['quaternion']).as_matrix(),poses[b['name']][:3,:3]),b['name']
    for link in robot.findall('link'):
        name = link.get('name')
        # Wheels and IMU are covered by the chassis/track envelopes rather
        # than separate boxes; independently verify their volumes as well.
        boxes = ([b for b in model['boxes'] if b.get('source_link', b['name']) == name]
                 if name in source_names else model['boxes'])
        for visual in robot.findall(f"link[@name='{name}']/visual"):
            t = poses[name]@transform(visual.find('origin'))
            shape = visual.find('geometry/box')
            if shape is not None:
                half = np.fromstring(shape.get('size'), sep=' ')/2
            else:
                shape = visual.find('geometry/cylinder')
                half = np.array([float(shape.get('radius'))]*2+[float(shape.get('length'))/2])
            points = (samples*half)@t[:3, :3].T+t[:3, 3]
            covered = np.zeros(len(points), dtype=bool)
            for b in boxes:
                local = (points-b['center'])@Rotation.from_quat(b['quaternion']).as_matrix()
                covered |= np.all(abs(local) <= np.array(b['size'])/2+1e-7, axis=1)
            assert covered.all(), (name, points[~covered])


def test_bucket_is_horizontal_and_small_arm_does_not_fill_one_large_void():
    model = yaml.safe_load((ROOT/'config/robot_model.yaml').read_text())
    bucket = next(b for b in model['boxes'] if b['name'] == 'link7')
    assert np.allclose(Rotation.from_quat(bucket['quaternion']).as_matrix(), np.eye(3))
    assert bucket['center'][2]-bucket['size'][2]/2 > .45
    assert bucket['size'][2] < .70
    arm = [b for b in model['boxes'] if b.get('source_link') == 'link4']
    assert len(arm) == 4
    assert sum(np.prod(b['size']) for b in arm) < .8*(3.130146*1.767111*.838902)


def test_drive_reversal_and_watchdog_braking_keep_acceleration_and_jerk_bounded():
    spec = importlib.util.spec_from_file_location('drive_s_curve', SIM/'scripts/clearing_tracks_drive.py')
    drive = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(drive)
    axis = drive.JerkLimitedAxis()
    dt, amax, jmax = .02, .3, .6
    history = []
    for target in [0.35]*250+[-.2]*350+[0.]*200:
        v, a = axis.velocity, axis.acceleration
        axis.advance(target, amax, jmax, dt)
        assert abs(axis.velocity-v) <= amax*dt+1e-9
        assert abs(axis.acceleration-a) <= jmax*dt+1e-9
        assert abs(axis.acceleration) <= amax+1e-9
        history.append(axis.velocity)
    assert max(history) > .34 and min(history) < -.19
    assert abs(axis.velocity)+abs(axis.acceleration) < 1e-8

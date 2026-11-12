#!/usr/bin/env python3
"""Check the navigation envelope, physical parameters, and control wiring."""

import importlib.util
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'clearing_tracks_drive', PACKAGE / 'scripts' / 'clearing_tracks_drive.py')
drive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(drive)


def numbers(text):
    return np.array([float(value) for value in text.split()])


def transform(origin):
    result = np.eye(4)
    if origin is not None:
        result[:3, 3] = numbers(origin.get('xyz', '0 0 0'))
        roll, pitch, yaw = numbers(origin.get('rpy', '0 0 0'))
        cr, cp, cy = np.cos([roll, pitch, yaw])
        sr, sp, sy = np.sin([roll, pitch, yaw])
        result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]]
    return result


def geometry_bounds(part, link_transform):
    pose = link_transform @ transform(part.find('origin'))
    geometry = part.find('geometry')
    box = geometry.find('box')
    if box is not None:
        extent = abs(pose[:3, :3]) @ (numbers(box.get('size')) / 2)
    else:
        cylinder = geometry.find('cylinder')
        radius = float(cylinder.get('radius'))
        half_length = float(cylinder.get('length')) / 2
        axis = pose[:3, 2]
        extent = radius * np.sqrt(np.maximum(0.0, 1.0 - axis**2)) + half_length * abs(axis)
    return pose[:3, 3] - extent, pose[:3, 3] + extent


class ClearingRobotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ET.fromstring(xacro.process_file(
            str(PACKAGE / 'robot' / 'clearing_robot_primitives.xacro')).toxml())
        cls.world = ET.parse(
            PACKAGE / 'worlds' / 'clearing_robot' / 'clearing_enclosed.world').getroot().find('world')
        cls.controllers = yaml.safe_load(
            (PACKAGE / 'config' / 'clearing_robot_primitives.yaml').read_text())
        cls.pose = dict(cls.controllers['robot_geometry']['ros__parameters'])
        cls.pose['navigation_footprint'] = json.loads(cls.pose['navigation_footprint'])
        cls.pose['stowed_positions'] = cls.controllers['initialize_clearing_arm']['ros__parameters']['positions']
        cls.original = ET.fromstring(xacro.process_file(
            str(PACKAGE / 'robot' / 'clearing_robot.xacro')).toxml())
        angles = dict(zip([f'joint{i}' for i in range(1, 8)], cls.pose['stowed_positions']))
        cls.transforms = {'base_link': np.eye(4)}
        pending = list(cls.robot.findall('joint'))
        while pending:
            unresolved = []
            for joint in pending:
                parent = joint.find('parent').get('link')
                if parent not in cls.transforms:
                    unresolved.append(joint)
                    continue
                rotation = np.eye(4)
                if joint.get('name') in angles:
                    # All seven original arm joints rotate about their local Z.
                    angle = angles[joint.get('name')]
                    c, s = np.cos(angle), np.sin(angle)
                    rotation[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
                cls.transforms[joint.find('child').get('link')] = (
                    cls.transforms[parent] @ transform(joint.find('origin')) @ rotation)
            if len(unresolved) == len(pending):
                raise AssertionError('Disconnected URDF joint tree')
            pending = unresolved

    def test_no_meshes_and_positive_inertias(self):
        self.assertFalse(self.robot.findall('.//mesh'))
        self.assertFalse(self.world.findall('.//mesh'))
        total_mass = 0.0
        for inertial in self.robot.findall('link/inertial'):
            mass = float(inertial.find('mass').get('value'))
            self.assertGreater(mass, 0)
            total_mass += mass
            i = {key: float(value) for key, value in inertial.find('inertia').attrib.items()}
            eigenvalues = np.linalg.eigvalsh([[i['ixx'], i['ixy'], i['ixz']],
                                            [i['ixy'], i['iyy'], i['iyz']],
                                            [i['ixz'], i['iyz'], i['izz']]])
            self.assertGreater(eigenvalues[0], 0)
            self.assertLessEqual(eigenvalues[2], sum(eigenvalues[:2]) + 1e-8)
        self.assertGreater(total_mass, 20000)
        self.assertLess(total_mass, 30000)

    def test_folded_arm_inside_navigation_footprint(self):
        footprint = np.array(self.pose['navigation_footprint'])
        for link in self.robot.findall('link'):
            if link.get('name') not in [f'link{index}' for index in range(1, 8)]:
                continue
            for part in link.findall('visual') + link.findall('collision'):
                lower, upper = geometry_bounds(part, self.transforms[link.get('name')])
                self.assertTrue(np.all(lower[:2] >= footprint.min(axis=0)), (link.get('name'), lower))
                self.assertTrue(np.all(upper[:2] <= footprint.max(axis=0)), (link.get('name'), upper))
                self.assertGreater(lower[2], self.pose['minimum_arm_ground_clearance'])
                self.assertLess(upper[2], self.pose['maximum_arm_height'])
                if link.get('name') == 'link7':
                    self.assertLess(upper[2], self.pose['maximum_bucket_height'])

    def test_hardware_and_controllers_agree(self):
        hardware = {joint.get('name'): joint for joint in self.robot.findall('ros2_control/joint')}
        wheels = self.controllers['tracks_velocity_controller']['ros__parameters']['joints']
        arm = self.controllers['arm_controller']['ros__parameters']['joints']
        self.assertEqual(len(wheels), 10)
        self.assertEqual(wheels[:5], [f'left_wheel_{label}_joint' for label in
                                     ('back', 'middle2', 'middle', 'middle1', 'front')])
        self.assertEqual(wheels[5:], [name.replace('left_', 'right_') for name in wheels[:5]])
        self.assertEqual(arm, [f'joint{index}' for index in range(1, 8)])
        self.assertEqual(set(hardware), set(wheels + arm))
        for name in wheels + arm:
            interface = hardware[name].find('command_interface').get('name')
            self.assertEqual(interface, 'velocity' if name in wheels else 'position')
        for name, target in zip(arm, self.pose['stowed_positions']):
            self.assertEqual(float(hardware[name].find("state_interface[@name='position']/param").text), target)
            limit = self.robot.find(f"joint[@name='{name}']/limit")
            self.assertLessEqual(float(limit.get('lower')), target)
            self.assertGreaterEqual(float(limit.get('upper')), target)
        types = [value['type'] for value in self.controllers['controller_manager']['ros__parameters'].values()
                 if isinstance(value, dict) and 'type' in value]
        self.assertFalse(any('DiffDrive' in kind for kind in types))
        self.assertIsNone(self.robot.find("gazebo/plugin[@filename='libgazebo_ros_p3d.so']"))

    def test_five_directional_vendor_avias_and_rigid_imu(self):
        self.assertIsNone(self.robot.find("link[@name='lidar_mount']"))
        self.assertIsNone(self.robot.find("link[@name='lidar_link']"))
        self.assertEqual(len(self.robot.findall("gazebo/sensor[@type='ray']")), 5)
        for index in range(1, 6):
            joint = self.robot.find(f"joint[@name='rslidar_{index}_joint']")
            original = self.original.find(f"joint[@name='rslidar_{index}_joint']")
            self.assertEqual(joint.find('parent').get('link'), 'link1')
            # Preserve headings while moving the four occluded mounts outboard.
            expected_rotation = transform(original.find('origin'))[:3, :3]
            if index in (3, 4):
                yaw = np.pi/18 * (1 if index == 3 else -1)
                expected_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                              [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
            np.testing.assert_allclose(transform(joint.find('origin'))[:3, :3], expected_rotation, atol=1e-12)
            sensor = self.robot.find(f"gazebo[@reference='rslidar_{index}']/sensor")
            plugin = sensor.find('plugin')
            self.assertEqual(plugin.get('filename'), 'libros2_livox.so')
            self.assertEqual(plugin.find('topic').text, f'/livox/lidar_{index}')
            self.assertTrue(Path(plugin.find('csv_file_name').text).is_file())
            preserved = self.robot.find(f"gazebo[@reference='rslidar_{index}_joint']/preserveFixedJoint")
            self.assertEqual(preserved.text, 'true')
        imu = self.robot.find("joint[@name='imu_joint']")
        self.assertEqual(imu.find('parent').get('link'), 'link1')
        imu_transform = transform(imu.find('origin'))
        primary = transform(self.robot.find("joint[@name='rslidar_3_joint']/origin"))
        relative = np.linalg.inv(imu_transform) @ primary
        parameters = self.controllers['fastlio_mapping']['ros__parameters']
        np.testing.assert_allclose(relative[:3, 3], parameters['mapping']['extrinsic_T'], atol=1e-12)
        np.testing.assert_allclose(relative[:3, :3],
                                   np.array(parameters['mapping']['extrinsic_R']).reshape(3, 3), atol=1e-12)
        self.assertEqual(parameters['common']['imu_topic'], '/livox/imu_raw')
        self.assertEqual(parameters['preprocess']['lidar_type'], 1)

    def test_front_lidars_overlap_ahead_of_the_arm(self):
        for index in (3, 4):
            mount = transform(self.robot.find(f"joint[@name='rslidar_{index}_joint']/origin"))
            # Target 8 m ahead of link1, beyond the arm. Both front sensors
            # must cover it, not only outward-facing side regions.
            target = np.linalg.inv(mount) @ np.array([8.0, 0.0, -0.6, 1.0])
            self.assertLess(abs(np.arctan2(target[1], target[0])), np.deg2rad(35.2))

    def test_downward_scan_origins_clear_the_platform(self):
        bounds = [geometry_bounds(part, np.eye(4))
                  for part in self.robot.findall("link[@name='link1']/collision")]
        lower = np.min([bound[0] for bound in bounds], axis=0)
        upper = np.max([bound[1] for bound in bounds], axis=0)
        positions = {index: transform(self.robot.find(
            f"joint[@name='rslidar_{index}_joint']/origin"))[:3, 3]
            for index in range(1, 6)}
        self.assertLessEqual(positions[1][0], lower[0] + 1e-6)
        for index in (2, 3):
            self.assertGreater(positions[index][1], upper[1] + 0.1)
        for index in (4, 5):
            self.assertLess(positions[index][1], lower[1] - 0.1)
        for index in (3, 4):
            self.assertGreater(positions[index][0], upper[0] + 0.1)

    def test_original_arm_joint_origins_axes_and_angle_limits_are_preserved(self):
        for index in range(1, 8):
            joint = self.robot.find(f"joint[@name='joint{index}']")
            original = self.original.find(f"joint[@name='joint{index}']")
            np.testing.assert_allclose(transform(joint.find('origin')),
                                       transform(original.find('origin')), atol=1e-12)
            np.testing.assert_array_equal(numbers(joint.find('axis').get('xyz')),
                                          numbers(original.find('axis').get('xyz')))
            for boundary in ('lower', 'upper'):
                self.assertEqual(float(joint.find('limit').get(boundary)),
                                 float(original.find('limit').get(boundary)))
            for connection in ('parent', 'child'):
                self.assertEqual(joint.find(connection).get('link'), original.find(connection).get('link'))
        self.assertIsNone(self.robot.find("link[@name='cab_link']"))

    def test_carry_pose_centre_of_mass_is_inside_track_support(self):
        total_mass = 0.0
        first_moment = np.zeros(3)
        for link in self.robot.findall('link'):
            inertial = link.find('inertial')
            if inertial is None:
                continue
            mass = float(inertial.find('mass').get('value'))
            centre = (self.transforms[link.get('name')] @ transform(inertial.find('origin')))[:3, 3]
            first_moment += mass * centre
            total_mass += mass
        centre = first_moment / total_mass
        # Wheel contact centres span x=[-2,2], y=[-1.4,1.4]. Reserve
        # a margin rather than accepting a static projection on the edge.
        self.assertLess(abs(centre[0]), 1.7)
        self.assertLess(abs(centre[1]), 1.1)
        self.assertLess(centre[2], 1.8)

    def test_original_component_dimensions_are_preserved(self):
        # Read CAD only as a test reference. The runtime xacro must remain
        # entirely independent of these STL files.
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        for index in range(1, 8):
            name = f'link{index}'
            mesh = self.original.find(f"link[@name='{name}']/visual/geometry/mesh")
            payload = Path(mesh.get('filename').removeprefix('file://')).read_bytes()
            points = np.frombuffer(payload, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
            link = self.robot.find(f"link[@name='{name}']")
            bounds = [geometry_bounds(part, np.eye(4)) for part in link.findall('visual')]
            lower = np.min([bound[0] for bound in bounds], axis=0)
            upper = np.max([bound[1] for bound in bounds], axis=0)
            np.testing.assert_allclose(lower, points.min(axis=0), atol=0.11, err_msg=name)
            np.testing.assert_allclose(upper, points.max(axis=0), atol=0.11, err_msg=name)

    def test_non_adjacent_arm_collision_envelopes_do_not_intersect_when_stowed(self):
        boxes = []
        for index in range(8):
            name = 'base_link' if index == 0 else f'link{index}'
            for collision in self.robot.findall(f"link[@name='{name}']/collision"):
                box = collision.find('geometry/box')
                cylinder = collision.find('geometry/cylinder')
                if box is not None:
                    half = numbers(box.get('size')) / 2
                else:
                    radius = float(cylinder.get('radius'))
                    half = np.array([radius, radius, float(cylinder.get('length')) / 2])
                pose = self.transforms[name] @ transform(collision.find('origin'))
                boxes.append((index, pose[:3, 3], pose[:3, :3], half))
        for offset, (index, p, r, half) in enumerate(boxes):
            for other, q, s, other_half in boxes[offset + 1:]:
                if abs(index - other) <= 1:
                    continue
                axes = np.concatenate((r.T, s.T, np.cross(r.T[:, None, :], s.T[None, :, :]).reshape(-1, 3)))
                axes = axes[np.linalg.norm(axes, axis=1) > 1e-7]
                overlap = np.abs(axes @ r) @ half + np.abs(axes @ s) @ other_half - np.abs(axes @ (q-p))
                self.assertFalse(np.all(overlap > 1e-5), (index, other))

    def test_world_encloses_exact_clear_volume(self):
        expected_faces = {'floor': (2, 0.0, 1), 'ceiling': (2, 20.0, -1),
                          'wall_west': (0, -25.0, 1), 'wall_east': (0, 25.0, -1),
                          'wall_south': (1, -25.0, 1), 'wall_north': (1, 25.0, -1)}
        for name, (axis, boundary, sign) in expected_faces.items():
            model = self.world.find(f"model[@name='{name}']")
            self.assertEqual(model.find('static').text, 'true')
            position = numbers(model.find('pose').text)[:3]
            size = numbers(model.find('link/collision/geometry/box/size').text)
            self.assertAlmostEqual(position[axis] + sign * size[axis]/2, boundary)
        # The default robot footprint has a generous clear spawn area.
        for model in self.world.findall('model'):
            if model.get('name') in expected_faces:
                continue
            x, y = numbers(model.find('pose').text)[:2]
            self.assertGreater(np.hypot(x, y), 12.0)

    def test_differential_commands(self):
        forward = drive.track_velocities(0.48, 0, 0.48, 2.8, 5, 8)
        self.assertEqual(forward, [1.0] * 10)
        turn = drive.track_velocities(0, 0.2, 0.48, 2.8, 5, 8)
        self.assertLess(turn[0], 0)
        self.assertAlmostEqual(turn[0], -turn[-1])
        self.assertEqual(drive.track_velocities(-0.48, 0, 0.48, 2.8, 5, 8), [-1.0] * 10)
        mixed = drive.track_velocities(1, 0.35, 0.48, 2.8, 5, 1)
        self.assertAlmostEqual(max(abs(value) for value in mixed), 1.0)
        self.assertAlmostEqual(mixed[0] / mixed[-1], (1-0.35*1.4)/(1+0.35*1.4))

    def test_drive_geometry_and_speed_limits_match_the_model(self):
        parameters = self.controllers['clearing_tracks_drive']['ros__parameters']
        left = self.robot.find("joint[@name='left_wheel_middle_joint']")
        right = self.robot.find("joint[@name='right_wheel_middle_joint']")
        spacing = abs(numbers(left.find('origin').get('xyz'))[1] -
                      numbers(right.find('origin').get('xyz'))[1])
        self.assertAlmostEqual(spacing, parameters['wheel_separation'])
        wheel = self.robot.find("link[@name='left_wheel_middle']/collision/geometry/cylinder")
        self.assertAlmostEqual(float(wheel.get('radius')), parameters['wheel_radius'])
        count = len([j for j in self.robot.findall('joint') if j.get('name').startswith('left_wheel_')])
        self.assertEqual(count, parameters['wheels_per_side'])
        velocities = drive.track_velocities(
            parameters['max_linear_velocity'], parameters['max_angular_velocity'],
            parameters['wheel_radius'], spacing, count, parameters['max_wheel_velocity'],
            parameters['steering_separation_multiplier'])
        for value, joint in zip(velocities, self.controllers['tracks_velocity_controller']['ros__parameters']['joints']):
            limit = self.robot.find(f"joint[@name='{joint}']/limit")
            self.assertLessEqual(abs(value), float(limit.get('velocity')))
        nominal = drive.track_velocities(0, .2, .48, 2.8, 5, 8)
        corrected = drive.track_velocities(0, .2, .48, 2.8, 5, 8, 1.17)
        np.testing.assert_allclose(corrected, np.array(nominal)*1.17)

    def test_gazebo_contact_direction_tracks_the_axle(self):
        for joint in self.robot.findall('joint'):
            if joint.get('type') != 'continuous':
                continue
            name = joint.find('child').get('link')
            link = self.robot.find(f"link[@name='{name}']")
            collision = link.find('collision')
            # sdformat 9 recognizes unnamed or link-prefixed geometries as
            # belonging to this link when applying Gazebo contact extensions.
            self.assertTrue(not collision.get('name') or collision.get('name').startswith(name))
            settings = self.robot.find(f"gazebo[@reference='{name}']")
            direction = numbers(settings.find('fdir1').text)
            link_direction = transform(collision.find('origin'))[:3, :3] @ direction
            axle = numbers(joint.find('axis').get('xyz'))
            self.assertAlmostEqual(abs(float(np.dot(link_direction, axle))), 1.0)
            self.assertLess(float(settings.find('mu1').text), float(settings.find('mu2').text))


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])

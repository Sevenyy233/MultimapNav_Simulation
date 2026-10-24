#!/usr/bin/env python3
"""Verify the four public robot entries and their ROS parameter files."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import xacro
import yaml
from rclpy.parameter import Parameter


PACKAGE = Path(__file__).resolve().parents[1]
ROBOTS = ('robot', 'tracked_robot', 'clearing_robot', 'clearing_robot_primitives')


class RobotLayoutTests(unittest.TestCase):
    def test_one_model_launch_and_yaml_per_robot(self):
        self.assertEqual({path.stem for path in (PACKAGE / 'robot').glob('*.xacro')}, set(ROBOTS))
        self.assertEqual({path.name for path in (PACKAGE / 'launch').glob('*.py')},
                         {f'{name}.launch.py' for name in ROBOTS})
        self.assertEqual({path.name for path in (PACKAGE / 'config').iterdir()},
                         {f'{name}.yaml' for name in ROBOTS})
        self.assertEqual(len(list(PACKAGE.rglob('*.yaml'))), 4)

    def test_each_model_loads_its_matching_controller_config(self):
        for name in ROBOTS:
            with self.subTest(robot=name):
                robot = ET.fromstring(xacro.process_file(str(PACKAGE / 'robot' / f'{name}.xacro')).toxml())
                plugins = robot.findall("gazebo/plugin[@filename='libgazebo_ros2_control.so']")
                self.assertEqual(len(plugins), 1)
                parameters = plugins[0].find('parameters').text
                self.assertEqual(Path(parameters).name, f'{name}.yaml')
                self.assertTrue(Path(parameters).is_file())
                profile = yaml.safe_load((PACKAGE / 'config' / f'{name}.yaml').read_text())
                self.assertIn('controller_manager', profile)
                # Check ROS parameter value types in every node section,
                # including pose, drive and optional navigation parameters.
                def check_values(prefix, values):
                    for key, value in values.items():
                        if isinstance(value, dict):
                            check_values(prefix + key + '.', value)
                        else:
                            Parameter(prefix + key, value=value)
                for node, section in profile.items():
                    self.assertIn('ros__parameters', section)
                    check_values(node + '.', section['ros__parameters'])

    def test_all_four_launch_descriptions_load(self):
        for name in ROBOTS:
            with self.subTest(robot=name):
                spec = importlib.util.spec_from_file_location(
                    f'robot_launch_{name}', PACKAGE / 'launch' / f'{name}.launch.py')
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                description = module.generate_launch_description()
                self.assertTrue(description.entities)


if __name__ == '__main__':
    unittest.main()

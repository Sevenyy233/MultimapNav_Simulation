"""Profile selection must retain clearing-robot geometry, odometry and algorithms."""
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location(
    'clearing_configuration', ROOT/'clearing_robot_plugins/configuration.py')
CONFIG = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONFIG)


def documents():
    return [yaml.safe_load((ROOT/'config'/name).read_text())
            for name in ('navigation.yaml', 'modes.yaml')]


def test_selector_runs_before_any_navigation_server():
    _, modes = documents()
    manager = modes['lifecycle_manager']['ros__parameters']
    assert manager['automatic_mode_selection'] is True
    assert manager['node_names'][0] == 'navigation_mode_selector_server'
    assert modes['navigation_mode_selector_server']['ros__parameters']['forced_mode'] == 'auto'
    assert set(manager['profiles']) == set(CONFIG.MODES)


@pytest.mark.parametrize('mode', CONFIG.MODES)
def test_every_mode_keeps_robot_plugins_and_fast_lio(mode):
    robot, modes = CONFIG.expand_profiles(*documents(), '/tmp/clearing_navigation.xml')
    profile = modes['lifecycle_manager']['ros__parameters']['profiles'][mode]
    assert profile['global_planner_plugin_type'] == 'clearing_robot_plugins::PaperGlobalPlanner'
    assert profile['local_planner_plugin_type'] == 'clearing_robot_plugins::HybridLocalPlanner'
    assert profile['trajectory_planner_plugin_type'] == 'clearing_robot_plugins::MPPITrajectory'
    assert profile['localization_plugin_type'] == 'clearing_robot_plugins::MapCorrectedFastLio'
    assert profile['base_frame'] == 'base_link'
    assert profile['trajectory_odom_topic'] == '/odom'
    assert profile['trajectory_odom_twist_in_world_frame'] is False
    for node, base in CONFIG.PLANNERS.items():
        params = robot[node]['ros__parameters']
        assert params[f'{base}_{mode}']['map_mode'] == mode
        assert all(params[f'{base}_{mode}'][key] == value for key, value in params[base].items())
    recovery = robot['recovery_server']['ros__parameters'][profile['recovery_plugin_namespace']]
    assert recovery['map_mode'] == mode
    assert recovery['back_up']['distance'] == .6
    assert recovery['spin']['angle'] == .26


def test_profiles_do_not_share_mutable_robot_limits():
    original, modes = documents()
    robot, _ = CONFIG.expand_profiles(original, modes, '/tmp/clearing_navigation.xml')
    params = robot['trajectory_planner_server']['ros__parameters']
    params['clearing_mppi_octomap']['max_linear_velocity'] = .01
    assert params['clearing_mppi_gridmap']['max_linear_velocity'] == .35
    assert original['trajectory_planner_server']['ros__parameters']['clearing_mppi']['max_linear_velocity'] == .35


def test_force_argument_rejects_invalid_mode():
    for mode in ('auto',)+CONFIG.MODES:
        assert CONFIG.validate_forced_mode(mode) == mode
    with pytest.raises(ValueError):
        CONFIG.validate_forced_mode('grid_map')

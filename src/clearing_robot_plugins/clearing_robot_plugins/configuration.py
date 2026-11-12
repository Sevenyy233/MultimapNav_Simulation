"""Create mode-specific namespaces while keeping robot constraints in one place."""
from copy import deepcopy

MODES = ('costmap2d', 'gridmap', 'octomap')
PLANNERS = {
    'global_planner_server': 'paper_astar',
    'local_planner_server': 'clearing_hybrid',
    'trajectory_planner_server': 'clearing_mppi',
}


def validate_forced_mode(value):
    if value not in ('auto',) + MODES:
        raise ValueError('forced_mode must be auto/costmap2d/gridmap/octomap')
    return value


def expand_profiles(robot, modes, bt_xml):
    """Return fresh YAML dictionaries; never import tracked_robot profiles."""
    robot, modes = deepcopy(robot), deepcopy(modes)
    manager = modes['lifecycle_manager']['ros__parameters']
    for mode in MODES:
        profile = manager['profiles'][mode]
        profile['bt_xml'] = str(bt_xml)
        for node, base in PLANNERS.items():
            parameters = robot[node]['ros__parameters']
            parameters[f'{base}_{mode}'] = dict(deepcopy(parameters[base]), map_mode=mode)
        recovery = robot['recovery_server']['ros__parameters']
        recovery[f'clearing_recovery_{mode}'] = {
            'map_mode': mode,
            **{name: deepcopy(recovery[name]) for name in recovery['recovery_plugins']},
        }
    return robot, modes


def node_parameters(document, name):
    result = deepcopy(document.get('/**', {}).get('ros__parameters', {}))
    result.update(deepcopy(document.get(name, {}).get('ros__parameters', {})))
    return result

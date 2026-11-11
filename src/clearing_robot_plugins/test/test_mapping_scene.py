"""A scene can resolve correctly while its default pose falls into an L-shaped void."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[1]
SIMULATION = ROOT.parent/'multimap_nav_simulation'
SPEC = importlib.util.spec_from_file_location('mapping_scene', ROOT/'clearing_robot_plugins/mapping_scene.py')
SCENE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENE)


@pytest.mark.parametrize('name,pose', [
    ('flat_4', [0, 3, .03, 0]),  # Empty quadrant of the L-shaped footprint.
    ('flat_3', [-9, 0, .03, 0]),  # Center is inside, part of a track is outside.
])
def test_unsupported_track_spawn_is_rejected(name, pose):
    with pytest.raises(ValueError, match='track area'):
        SCENE.validate_catalog_spawn(SIMULATION, SIMULATION/f'worlds/flat/{name}.world',
                                     pose, ROOT/'config/robot_model.yaml')


def test_plaza_pose_fits_track_support_boundary():
    SCENE.validate_catalog_spawn(SIMULATION, SIMULATION/'worlds/flat/flat_3.world',
                                 [-6, -7, .03, 0], ROOT/'config/robot_model.yaml')


def test_custom_world_does_not_inherit_same_named_catalog_boundary(tmp_path):
    SCENE.validate_catalog_spawn(SIMULATION, tmp_path/'flat_4.world',
                                 [0, 3, .03, 0], ROOT/'config/robot_model.yaml')

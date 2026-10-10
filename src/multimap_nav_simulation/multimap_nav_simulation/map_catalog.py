"""Resolve categorized map names and their documented spawn positions."""

import os
from pathlib import Path

import yaml


ALIASES = {
    "indoor_flat": "flat_1", "flat_indoor": "flat_1",
    "terrain": "terrain_1", "terrain_with_wall": "terrain_1",
    "two_floor": "floor_1", "two_floior": "floor_1",
}


def load_catalog(package_dir):
    return yaml.safe_load((Path(package_dir) / "config/maps.yaml").read_text())["maps"]


def resolve_world(package_dir, name):
    package_dir = Path(package_dir)
    path = Path(os.path.expanduser(name.strip()))
    if path.is_absolute():
        path = path.with_suffix(".world")
    else:
        key = ALIASES.get(path.stem, path.stem)
        catalog = load_catalog(package_dir)
        path = (package_dir / catalog[key]["world"] if key in catalog
                else package_dir / "worlds" / path.with_suffix(".world"))
    if not path.is_file():
        raise FileNotFoundError(f"World does not exist: {path}; choose flat_1..5, terrain_1..5 or floor_1..7")
    return path


def spawn_pose(package_dir, world, level="lower", robot="tracked"):
    record = load_catalog(package_dir).get(Path(world).stem, {})
    if level == "upper":
        if "upper_spawn" not in record:
            raise ValueError(f"Map {Path(world).stem} has no upper floor")
        return record["upper_spawn"]
    if level != "lower":
        raise ValueError("spawn_level must be lower or upper")
    return record.get("clearing_spawn" if robot == "clearing" else "spawn", record.get("spawn", [0, 0, .08, 0]))


def _path_env(*parts):
    seen = []
    for part in parts:
        for item in str(part).split(os.pathsep):
            if item and item not in seen:
                seen.append(item)
    return os.pathsep.join(seen)


def gazebo_environment(package_dir):
    package_dir = Path(package_dir)
    # Gazebo Classic looks up relative mesh URIs in GAZEBO_RESOURCE_PATH and the
    # process cwd, not next to the world file. Terrain/floor meshes are stored as
    # 3DModel/... under the package share directory. models/ still resolves
    # bundled furniture. An empty database URI prevents hanging on Fuel.
    from ament_index_python.packages import get_package_prefix
    plugin_path=str(Path(get_package_prefix("multimap_nav_simulation"))/"lib")
    return {
        "GAZEBO_PLUGIN_PATH": plugin_path + os.pathsep + os.environ.get("GAZEBO_PLUGIN_PATH", ""),
        "GAZEBO_MODEL_PATH": _path_env(
            package_dir / "models", "/usr/share/gazebo-11/models",
            os.environ.get("GAZEBO_MODEL_PATH", "")),
        "GAZEBO_RESOURCE_PATH": _path_env(
            package_dir, "/usr/share/gazebo-11",
            os.environ.get("GAZEBO_RESOURCE_PATH", "")),
        "GAZEBO_MODEL_DATABASE_URI": "",
    }

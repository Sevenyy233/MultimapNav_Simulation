"""Check known scene boundaries using metadata, without reading a prior PCD."""
import json
import math
from pathlib import Path

import yaml


def inside_polygon(x, y, polygon):
    inside = False
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        cross = (x-a[0])*dy-(y-a[1])*dx
        if (abs(cross) < 1e-8 and min(a[0], b[0])-1e-8 <= x <= max(a[0], b[0])+1e-8
                and min(a[1], b[1])-1e-8 <= y <= max(a[1], b[1])+1e-8):
            return True
        if (a[1] > y) != (b[1] > y) and x < dx*(y-a[1])/dy+a[0]:
            inside = not inside
    return inside


def validate_catalog_spawn(simulation, world, pose, model_file):
    simulation, world = Path(simulation), Path(world)
    record = json.loads((simulation/'worlds/catalog.json').read_text())['maps'].get(world.stem)
    if not record or world.resolve() != (simulation/record['world']).resolve():
        return  # Custom worlds have no catalog boundary to check.
    polygon = record['footprint_polygon']
    c, s = math.cos(pose[3]), math.sin(pose[3])
    boxes = yaml.safe_load(Path(model_file).read_text())['boxes']
    for box in boxes:
        if not box['name'].endswith('track_belt'):
            continue
        nx = max(1, math.ceil(box['size'][0]/.4))
        ny = max(1, math.ceil(box['size'][1]/.2))
        for i in range(nx+1):
            for j in range(ny+1):
                x = box['center'][0]+box['size'][0]*(i/nx-.5)
                y = box['center'][1]+box['size'][1]*(j/ny-.5)
                if not inside_polygon(pose[0]+c*x-s*y, pose[1]+s*x+c*y, polygon):
                    raise ValueError(f'{world.stem}: track area lies outside the scene ground '
                                     'boundary. Set spawn_x/spawn_y/spawn_yaw to a supported pose.')

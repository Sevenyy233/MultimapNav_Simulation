#!/usr/bin/env python3
"""Rebuild the 15-map suite, its world-coordinate meshes and navigation clouds.

The three *_1 worlds/terrain mesh are retained baselines. Maps 2-5 use Gazebo
primitives and deterministic triangulated height surfaces. Layouts keep 1.2 m
aisles for tracked_robot.xacro (about 0.40 m x 0.40 m).
"""

from __future__ import annotations

import argparse
import math
import shutil
from collections import deque
from pathlib import Path

import numpy as np
import xml.etree.ElementTree as ET
import yaml

from stl_to_navigation_pcd import (interior_from_size, read_binary_stl,
                                   sample_navigation_surfaces, write_binary_pcd)
from world_to_mesh import (box_triangles, cylinder_triangles, pose_matrix, sphere_triangles,
                           transform, world_triangles, write_stl)


PACKAGE_DIR = Path(__file__).resolve().parents[1]
ROBOT_INFLATE = 0.28
PALETTE = {}
PALETTES = {
    "flat": {
        "ambient": "0.50 0.49 0.47 1", "background": "0.80 0.82 0.86 1",
        "ground": "0.78 0.74 0.68 1", "wall": "0.90 0.91 0.93 1",
        "structure": "0.28 0.46 0.66 1", "obstacle": "0.62 0.40 0.24 1",
        "accent": "0.16 0.52 0.50 1", "prop": "0.46 0.50 0.54 1",
    },
    "terrain": {
        "ambient": "0.52 0.56 0.48 1", "background": "0.58 0.76 0.92 1",
        "ground": "0.38 0.54 0.24 1", "wall": "0.62 0.58 0.50 1",
        "structure": "0.52 0.44 0.34 1", "obstacle": "0.50 0.34 0.20 1",
        "accent": "0.66 0.50 0.28 1", "prop": "0.42 0.36 0.30 1",
    },
    "floor": {
        "ambient": "0.48 0.49 0.53 1", "background": "0.72 0.77 0.84 1",
        "ground": "0.54 0.57 0.62 1", "upper": "0.86 0.81 0.62 1",
        "wall": "0.91 0.91 0.93 1", "structure": "0.26 0.40 0.58 1",
        "obstacle": "0.60 0.36 0.22 1", "accent": "0.92 0.55 0.16 1",
        "prop": "0.40 0.44 0.50 1", "lower_terrain": "0.44 0.52 0.32 1",
        "upper_terrain": "0.72 0.48 0.30 1", "ramp": "0.92 0.62 0.20 1",
    },
}
BASELINE_MATERIALS = {
    "Gazebo/Grey": "ground", "Gazebo/White": "wall", "Gazebo/Blue": "structure",
    "Gazebo/Yellow": "upper", "Gazebo/Orange": "accent", "Gazebo/DarkGrey": "prop",
}


def text(parent, tag, value):
    ET.SubElement(parent, tag).text = str(value)


def vec(values):
    return " ".join(f"{v:.9g}" for v in values)


def mesh_uri(*parts):
    """Package-share-relative mesh path; launch puts the share dir on GAZEBO_RESOURCE_PATH."""
    return str(Path("3DModel", *parts).as_posix())


def save_xml(path, root):
    ET.indent(root, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    with path.open("ab") as stream:
        stream.write(b"\n")


def new_world(name, size):
    global PALETTE
    category = name.split("_")[0]
    PALETTE = PALETTES[category]
    root = ET.Element("sdf", version="1.7")
    world = ET.SubElement(root, "world", name=name)
    text(world, "gravity", "0 0 -9.8")
    plugin = ET.SubElement(world, "plugin", name="gazebo_ros_state", filename="libgazebo_ros_state.so")
    ros = ET.SubElement(plugin, "ros")
    text(ros, "namespace", "/gazebo")
    physics = ET.SubElement(world, "physics", name="ode_physics", type="ode")
    for tag, value in (("max_step_size", 0.001), ("real_time_factor", 1), ("real_time_update_rate", 1000)):
        text(physics, tag, value)
    ode = ET.SubElement(physics, "ode")
    solver = ET.SubElement(ode, "solver")
    for tag, value in (("type", "quick"), ("iters", 150), ("sor", 1.3)):
        text(solver, tag, value)
    constraints = ET.SubElement(ode, "constraints")
    for tag, value in (("cfm", 0), ("erp", 0.2), ("contact_max_correcting_vel", 0.2),
                       ("contact_surface_layer", 0.001)):
        text(constraints, tag, value)
    light = ET.SubElement(world, "light", name="sun", type="directional")
    sun_z = 42 if category == "terrain" else 28
    for tag, value in (("pose", f"0 0 {sun_z} 0 0 0"), ("diffuse", "0.92 0.91 0.88 1"),
                       ("specular", "0.18 0.18 0.16 1"), ("direction", "-0.35 -0.28 -0.9"),
                       ("cast_shadows", "true")):
        text(light, tag, value)
    fill = ET.SubElement(world, "light", name="fill", type="point")
    text(fill, "pose", f"0 0 {6 if category != 'floor' else 4.5} 0 0 0")
    text(fill, "diffuse", "0.45 0.48 0.52 1" if category == "floor" else "0.35 0.36 0.34 1")
    text(fill, "specular", "0.02 0.02 0.02 1")
    text(fill, "cast_shadows", "false")
    attenuation = ET.SubElement(fill, "attenuation")
    text(attenuation, "range", 80)
    text(attenuation, "constant", 0.5)
    text(attenuation, "linear", 0.02)
    scene = ET.SubElement(world, "scene")
    text(scene, "ambient", PALETTE["ambient"])
    text(scene, "background", PALETTE["background"])
    gui = ET.SubElement(world, "gui", fullscreen="false")
    camera = ET.SubElement(gui, "camera", name="user_camera")
    text(camera, "pose", vec((size[0] * 0.7, -size[1] * 0.9, max(size) * 0.85, 0, 0.65, 2.2)))
    text(camera, "view_controller", "orbit")
    model = ET.SubElement(world, "model", name=name + "_structure")
    text(model, "static", "true")
    link = ET.SubElement(model, "link", name="structure")
    return root, link


def add_shape(link, name, kind, values, pose=(0, 0, 0, 0, 0, 0), color="wall", collide=True):
    for tag in (["collision", "visual"] if collide else ["visual"]):
        node = ET.SubElement(link, tag, name=f"{name}_{tag}")
        text(node, "pose", vec(pose))
        geom = ET.SubElement(node, "geometry")
        shape = ET.SubElement(geom, kind)
        if kind == "box":
            text(shape, "size", vec(values))
        elif kind == "cylinder":
            text(shape, "radius", values[0]); text(shape, "length", values[1])
        elif kind == "sphere":
            text(shape, "radius", values[0] if isinstance(values, (tuple, list)) else values)
        elif kind == "mesh":
            text(shape, "uri", values)
        if tag == "visual":
            material = ET.SubElement(node, "material")
            rgba = PALETTE[color]
            text(material, "ambient", rgba); text(material, "diffuse", rgba)
        else:
            text(node, "max_contacts", 40)
            surface = ET.SubElement(node, "surface")
            friction = ET.SubElement(surface, "friction")
            ode = ET.SubElement(friction, "ode")
            text(ode, "mu", 8 if kind == "mesh" else 2)
            text(ode, "mu2", 8 if kind == "mesh" else 2)
            contact = ET.SubElement(ET.SubElement(surface, "contact"), "ode")
            for field, value in (("kp", "1e7"), ("kd", "1000"), ("max_vel", "0.1"), ("min_depth", "0.001")):
                text(contact, field, value)


def track(footprints, level, x, y, sx, sy, yaw=0):
    if footprints is not None:
        footprints.append((level, x, y, sx, sy, yaw))


def box(link, name, x, y, z, sx, sy, sz, color="wall", pitch=0, yaw=0, footprints=None, level="all"):
    add_shape(link, name, "box", (sx, sy, sz), (x, y, z, 0, pitch, yaw), color)
    track(footprints, level, x, y, sx, sy, yaw)


def cylinder(link, name, x, y, z, radius, length, color="prop", footprints=None, level="all"):
    add_shape(link, name, "cylinder", (radius, length), (x, y, z, 0, 0, 0), color)
    track(footprints, level, x, y, 2 * radius, 2 * radius, 0)


def sphere(link, name, x, y, z, radius, color="obstacle", footprints=None, level="all"):
    add_shape(link, name, "sphere", (radius,), (x, y, z, 0, 0, 0), color)
    track(footprints, level, x, y, 2 * radius, 2 * radius, 0)


def boundary(link, sx, sy, height=2.2, z=0, center=(0, 0), prefix="boundary", footprints=None):
    x, y = center
    box(link, prefix + "_west", x - sx / 2 - 0.1, y, z + height / 2, 0.2, sy + 0.4, height,
        footprints=footprints)
    box(link, prefix + "_east", x + sx / 2 + 0.1, y, z + height / 2, 0.2, sy + 0.4, height,
        footprints=footprints)
    box(link, prefix + "_south", x, y - sy / 2 - 0.1, z + height / 2, sx, 0.2, height,
        footprints=footprints)
    box(link, prefix + "_north", x, y + sy / 2 + 0.1, z + height / 2, sx, 0.2, height,
        footprints=footprints)


def smoothstep(value):
    value = np.clip(value, 0, 1)
    return value * value * (3 - 2 * value)


def heightfield(bounds, profile, base=0, step=0.35, thickness=0.25):
    xmin, xmax, ymin, ymax = bounds
    x, y = np.meshgrid(np.linspace(xmin, xmax, math.ceil((xmax - xmin) / step) + 1),
                       np.linspace(ymin, ymax, math.ceil((ymax - ymin) / step) + 1), indexing="ij")
    z = base + profile(x, y)
    vertices = np.stack((x, y, z), axis=-1)
    triangles = [
        np.stack((vertices[:-1, :-1], vertices[1:, :-1], vertices[1:, 1:]), axis=-2).reshape(-1, 3, 3),
        np.stack((vertices[:-1, :-1], vertices[1:, 1:], vertices[:-1, 1:]), axis=-2).reshape(-1, 3, 3),
    ]
    bottom_z = float(z.min()) - thickness
    loop = np.concatenate((vertices[:, 0], vertices[-1, 1:], vertices[-2::-1, -1], vertices[0, -2:0:-1]))
    for i in range(len(loop)):
        a, b = loop[i], loop[(i + 1) % len(loop)]
        c, d = b.copy(), a.copy(); c[2] = d[2] = bottom_z
        triangles.append(np.array([[a, d, c], [a, c, b]]))
    triangles.append(np.array([[[xmin, ymin, bottom_z], [xmin, ymax, bottom_z], [xmax, ymax, bottom_z]],
                               [[xmin, ymin, bottom_z], [xmax, ymax, bottom_z], [xmax, ymin, bottom_z]]]))
    return np.concatenate(triangles)


def primitive_triangles(link):
    chunks = []
    for collision in link.findall("collision"):
        geom = collision.find("geometry")
        pose = pose_matrix(collision.findtext("pose"))
        if geom.find("box") is not None:
            triangles = box_triangles(np.fromstring(geom.findtext("box/size"), sep=" "))
        elif geom.find("cylinder") is not None:
            triangles = cylinder_triangles(float(geom.findtext("cylinder/radius")),
                                           float(geom.findtext("cylinder/length")))
        elif geom.find("sphere") is not None:
            triangles = sphere_triangles(float(geom.findtext("sphere/radius")))
        else:
            continue
        chunks.append(transform(triangles, pose))
    return chunks


def mesh_support_z(triangles, x, y, limit=4.0):
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    det = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    valid = det > 1e-9
    u, v = np.zeros(len(triangles)), np.zeros(len(triangles))
    u[valid] = (((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1]))[valid] / det[valid])
    v[valid] = (((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1]))[valid] / det[valid])
    height = u * a[:, 2] + v * b[:, 2] + (1 - u - v) * c[:, 2]
    valid &= (u >= -1e-6) & (v >= -1e-6) & (u + v <= 1 + 1e-6) & (height < limit + .01)
    if not valid.any():
        return 0.0
    return float(height[valid].max())


def split_terrain_visuals(triangles):
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normal_z = cross[:, 2] / np.maximum(np.linalg.norm(cross, axis=1), 1e-12)
    surface = triangles[normal_z > 0.45]
    walls = triangles[normal_z <= 0.45]
    if len(surface) == 0:
        surface = triangles
    if len(walls) == 0:
        walls = triangles[:1]
    return surface, walls


def assert_connected(name, start, goals, footprints, interior, level="all"):
    xmin, xmax, ymin, ymax = interior
    res = 0.25
    nx = int(round((xmax - xmin) / res)) + 1
    ny = int(round((ymax - ymin) / res)) + 1
    occupied = np.zeros((nx, ny), dtype=bool)
    for item_level, x, y, sx, sy, yaw in footprints:
        if item_level not in (level, "all"):
            continue
        cosine, sine = math.cos(yaw), math.sin(yaw)
        half_x, half_y = sx / 2 + ROBOT_INFLATE, sy / 2 + ROBOT_INFLATE
        bound_x = half_x * abs(cosine) + half_y * abs(sine)
        bound_y = half_x * abs(sine) + half_y * abs(cosine)
        i0 = max(0, int(math.floor((x - bound_x - xmin) / res)))
        i1 = min(nx, int(math.ceil((x + bound_x - xmin) / res)) + 1)
        j0 = max(0, int(math.floor((y - bound_y - ymin) / res)))
        j1 = min(ny, int(math.ceil((y + bound_y - ymin) / res)) + 1)
        for i in range(i0, i1):
            for j in range(j0, j1):
                dx, dy = xmin + i * res - x, ymin + j * res - y
                local_x = cosine * dx + sine * dy
                local_y = -sine * dx + cosine * dy
                if abs(local_x) <= half_x and abs(local_y) <= half_y:
                    occupied[i, j] = True

    def cell(point):
        return (int(round((point[0] - xmin) / res)), int(round((point[1] - ymin) / res)))

    start_cell = cell(start)
    if not (0 <= start_cell[0] < nx and 0 <= start_cell[1] < ny) or occupied[start_cell]:
        raise AssertionError(f"{name}: spawn {start} is blocked")
    seen = np.zeros_like(occupied)
    queue = deque([start_cell])
    seen[start_cell] = True
    while queue:
        i, j = queue.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < nx and 0 <= nj < ny and not seen[ni, nj] and not occupied[ni, nj]:
                seen[ni, nj] = True
                queue.append((ni, nj))
    for goal in goals:
        gi, gj = cell(goal)
        found = None
        for radius in range(0, int(1.6 / res) + 1):
            for di in range(-radius, radius + 1):
                dj_remain = radius - abs(di)
                for dj in (-dj_remain, dj_remain) if dj_remain else (0,):
                    ni, nj = gi + di, gj + dj
                    if 0 <= ni < nx and 0 <= nj < ny and not occupied[ni, nj] and seen[ni, nj]:
                        found = (ni, nj)
                        break
                if found:
                    break
            if found:
                break
        if found is None:
            raise AssertionError(f"{name}: tracked robot cannot reach {goal} from {start}")


def flat_map(index):
    sizes = {2: (20, 16), 3: (22, 18), 4: (26, 22), 5: (24, 20)}
    size = sizes[index]
    spawn = (-size[0] / 2 + 1.0, -size[1] / 2 + 1.0)
    if index == 4:
        spawn = (-7.0, 0.0)
    root, link = new_world(f"flat_{index}", size)
    marks = []
    box(link, "floor", 0, 0, -0.1, *size, 0.2, color="ground")
    boundary(link, *size, footprints=marks)
    if index == 2:
        racks = [(-5.3, -3.1, 1.25, 3.4, 0.11), (-4.6, 3.6, 1.45, 3.1, -0.16),
                 (0.2, -2.7, 1.15, 4.0, 0.19), (0.7, 3.3, 1.35, 3.2, -0.08),
                 (5.6, -3.4, 1.2, 2.8, 0.24), (5.1, 3.7, 1.5, 3.6, -0.13)]
        for i, (x, y, sx, sy, yaw) in enumerate(racks):
            box(link, f"rack_{i}", x, y, 0.95, sx, sy, 1.9, "structure", yaw=yaw,
                footprints=marks, level="lower")
        for i, (x, y, yaw) in enumerate(((-7.2, 5.8, 0.4), (-2.4, 6.6, -0.35), (2.1, 6.3, 0.2),
                                         (6.4, 6.8, -0.5), (8.1, 4.2, 0.7))):
            box(link, f"pallet_{i}", x, y, 0.22, 1.15, 0.85, 0.44, "obstacle", yaw=yaw,
                footprints=marks, level="lower")
        for i, (x, y, radius) in enumerate(((8.2, -6.4, 0.28), (7.3, -5.5, 0.32), (8.6, -4.7, 0.24),
                                            (7.8, 1.1, 0.30), (-7.6, 2.4, 0.26), (2.8, -6.6, 0.29))):
            cylinder(link, f"barrel_{i}", x, y, 0.45, radius, 0.9, "accent",
                     footprints=marks, level="lower")
        for i, (x, y) in enumerate(((8.4, -0.8), (-6.8, -4.2))):
            cylinder(link, f"column_{i}", x, y, 1.15, 0.22, 2.3, "prop",
                     footprints=marks, level="lower")
        box(link, "loading_dock", 8.3, 6.9, 0.18, 2.4, 1.1, 0.36, "prop", yaw=-0.18,
            footprints=marks, level="lower")
        goals = [(0, 0), (8, -6), (8, 6), (-8, 6), (0, 6)]
    elif index == 3:
        # Asymmetric offices around a 3 m corridor; doorways stay at least 1.4 m.
        for i, (x, y, sx, sy) in enumerate((
                (-7.6, -4.6, 3.4, 0.16), (-2.4, -4.6, 3.0, 0.16), (3.3, -4.6, 3.6, 0.16),
                (8.4, -4.8, 2.6, 0.16), (-8.2, 4.8, 3.2, 0.16), (-2.8, 4.7, 2.8, 0.16),
                (3.8, 4.6, 4.0, 0.16), (8.6, 4.9, 2.4, 0.16))):
            box(link, f"room_front_{i}", x, y, 1.1, sx, sy, 2.2, "structure",
                footprints=marks, level="lower")
        for i, (x, y, sx, sy) in enumerate((
                (-5.1, -6.8, 0.16, 4.2), (0.6, -6.9, 0.16, 4.4), (6.2, -6.6, 0.16, 3.8),
                (-5.6, 6.7, 0.16, 3.6), (0.2, 6.9, 0.16, 4.0), (6.8, 6.6, 0.16, 3.4))):
            box(link, f"room_side_{i}", x, y, 1.1, sx, sy, 2.2, "wall",
                footprints=marks, level="lower")
        desks = [(-8.4, -6.9, 1.3, 0.7, 0.08), (-2.2, -7.1, 1.4, 0.75, -0.12),
                 (3.6, -6.8, 1.2, 0.7, 0.2), (8.5, -7.0, 1.35, 0.8, -0.18),
                 (-8.1, 7.0, 1.25, 0.7, 0.15), (-1.6, 6.8, 1.4, 0.8, -0.25),
                 (3.9, 7.2, 1.15, 0.7, 0.1), (8.3, 6.7, 1.3, 0.75, 0.3)]
        for i, (x, y, sx, sy, yaw) in enumerate(desks):
            box(link, f"desk_{i}", x, y, 0.4, sx, sy, 0.8, "obstacle", yaw=yaw,
                footprints=marks, level="lower")
        for i, (x, y) in enumerate(((-8.0, -5.8), (-1.4, -5.9), (4.2, 5.9), (8.0, 5.8),
                                    (8.6, -0.8), (-8.4, 0.6))):
            cylinder(link, f"chair_{i}", x, y, 0.28, 0.22, 0.56, "prop",
                     footprints=marks, level="lower")
        for i, (x, y) in enumerate(((-0.8, -7.8), (7.4, 7.8), (-7.2, 7.9), (2.4, 0.0))):
            cylinder(link, f"plant_{i}", x, y, 0.45, 0.18, 0.9, "accent",
                     footprints=marks, level="lower")
        box(link, "cabinet_a", -9.4, 1.8, 0.7, 0.9, 1.6, 1.4, "prop", yaw=0.12,
            footprints=marks, level="lower")
        box(link, "cabinet_b", 9.5, -2.2, 0.65, 0.8, 1.8, 1.3, "prop", yaw=-0.2,
            footprints=marks, level="lower")
        goals = [(0, 0), (9, 0), (-9, 7), (9, -7), (0, 7)]
    elif index == 4:
        pillars = [(-8.4, -6.2, 0.42), (-9.1, 6.8, 0.55), (0.8, -8.1, 0.38),
                   (-1.2, 8.4, 0.48), (7.6, -6.8, 0.44), (8.8, 5.9, 0.52),
                   (11.4, 0.6, 0.36), (-11.2, -3.4, 0.4)]
        for i, (x, y, radius) in enumerate(pillars):
            cylinder(link, f"pillar_{i}", x, y, 1.3, radius, 2.6, "structure",
                     footprints=marks, level="lower")
        planters = [(-3.6, 2.4, 2.8, 1.0, 0.42), (4.8, -2.2, 3.2, 1.15, -0.55),
                    (9.6, 2.8, 2.4, 1.05, 0.7), (-4.8, -7.4, 2.6, 0.95, 0.28)]
        for i, (x, y, sx, sy, yaw) in enumerate(planters):
            box(link, f"planter_{i}", x, y, 0.38, sx, sy, 0.76, "accent", yaw=yaw,
                footprints=marks, level="lower")
        for i, (x, y, yaw) in enumerate(((-11.2, 7.6, 0.15), (-10.6, -8.2, -0.2),
                                         (10.8, -8.6, 0.4), (3.2, 8.8, -0.3))):
            box(link, f"bench_{i}", x, y, 0.32, 1.8, 0.55, 0.64, "obstacle", yaw=yaw,
                footprints=marks, level="lower")
        for i, (x, y, radius) in enumerate(((2.6, 3.8, 0.32), (6.2, 7.1, 0.28),
                                            (-2.2, -3.6, 0.35), (11.2, 8.2, 0.26),
                                            (0.4, -4.8, 0.3), (-8.8, 2.2, 0.34))):
            sphere(link, f"stone_{i}", x, y, radius, radius, "prop",
                   footprints=marks, level="lower")
        box(link, "kiosk", 1.6, 0.8, 0.7, 1.4, 1.1, 1.4, "structure", yaw=-0.35,
            footprints=marks, level="lower")
        goals = [(0, 0), (10, 0), (10, 8), (-10, 8), (-6, -9), (8, -8)]
    else:
        walls = [(-7.2, -1.8, 0.18, 12.4, 0.04), (-2.4, 2.4, 0.18, 10.6, -0.05),
                 (2.6, -1.6, 0.18, 11.8, 0.07), (7.1, 1.8, 0.18, 9.6, -0.03),
                 (-9.6, 3.4, 4.0, 0.18, 0.08), (0.4, -6.6, 4.8, 0.18, -0.12),
                 (5.4, 7.1, 5.2, 0.18, 0.16), (9.4, -4.2, 3.6, 0.18, -0.2),
                 (-4.6, 7.6, 3.2, 0.18, 0.1)]
        for i, (x, y, sx, sy, yaw) in enumerate(walls):
            box(link, f"baffle_{i}", x, y, 1.1, sx, sy, 2.2, "structure", yaw=yaw,
                footprints=marks, level="lower")
        for i, (x, y, yaw) in enumerate(((-9.5, -4.8, 0.3), (0.8, 4.6, -0.4),
                                         (9.6, 6.2, 0.5), (4.2, -7.8, -0.25))):
            box(link, f"crate_{i}", x, y, 0.45, 0.9, 0.7, 0.9, "obstacle", yaw=yaw,
                footprints=marks, level="lower")
        for i, (x, y) in enumerate(((9.8, 1.2), (-0.6, -8.2), (-9.2, 6.8), (8.8, -7.6))):
            cylinder(link, f"barrel_{i}", x, y, 0.42, 0.26, 0.84, "accent",
                     footprints=marks, level="lower")
        sphere(link, "boulder_a", 4.8, 3.2, 0.28, 0.28, "prop", footprints=marks, level="lower")
        sphere(link, "boulder_b", -5.2, -7.2, 0.24, 0.24, "prop", footprints=marks, level="lower")
        goals = [(0, 0), (10, 0), (10, 8), (-10, 8), (0, 8), (8, -8)]
    interior = (-size[0] / 2 + 0.4, size[0] / 2 - 0.4, -size[1] / 2 + 0.4, size[1] / 2 - 0.4)
    assert_connected(f"flat_{index}", spawn, goals, marks, interior, "lower")
    return root, size


def terrain_profile(index, size, spawn):
    sx, sy = size

    def profile(x, y):
        if index == 2:
            z = (1.45 * np.exp(-((x + 4) ** 2 / 20 + (y - 3) ** 2 / 14))
                 + 1.1 * np.exp(-((x - 7) ** 2 / 12 + (y + 5) ** 2 / 22))
                 + .75 * np.exp(-((x + 8) ** 2 / 9 + (y + 5) ** 2 / 11)))
        elif index == 3:
            center = 2.2 * np.sin(y / 5)
            z = (1.65 * np.exp(-((x - center - 5) / 2.8) ** 2)
                 + 1.2 * np.exp(-((x - center + 6) / 3.2) ** 2))
            z *= .7 + .3 * np.cos(y / 7) ** 2
        elif index == 4:
            z = .30 + .17 * np.sin(x / 1.5) * np.cos(y / 2.1) + .14 * np.cos((x + y) / 2.3)
            for cx, cy in ((-6, -2), (1, 4), (7, -5)):
                z += .65 * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / 7)
        else:
            z = sum(.23 * (1 + np.tanh((x - center - .7 * np.sin(y / 4)) / 1.9))
                    for center in (-7, 0, 7))
            z += .12 * np.sin(y / 2.5) ** 2
        edge = smoothstep(np.minimum(sx / 2 - np.abs(x), sy / 2 - np.abs(y)) / 3.0)
        spawn_blend = smoothstep((np.hypot(x - spawn[0], y - spawn[1]) - 1.8) / 1.8)
        return np.maximum(z, 0) * edge * spawn_blend

    return profile


def terrain_map(index, package):
    sizes = {2: (34, 28), 3: (32, 30), 4: (36, 26), 5: (34, 30)}
    size = sizes[index]
    spawn = (-size[0] / 2 + 3, -size[1] / 2 + 3, .08, 0)
    profile = terrain_profile(index, size, spawn[:2])
    surface = heightfield((-size[0] / 2, size[0] / 2, -size[1] / 2, size[1] / 2), profile)
    surface_path = package / "3DModel/terrain/surfaces" / f"terrain_{index}_surface.stl"
    write_stl(surface_path, surface)
    root, link = new_world(f"terrain_{index}", size)
    marks = []
    boundary(link, *size, height=1.5, footprints=marks)
    obstacles = {
        2: [
            ("box", -1.4, -5.2, 1.3, 0.9, 0.9, 0.35),
            ("box", 6.8, 5.4, 1.7, 1.1, 0.85, -0.4),
            ("box", -8.6, 6.2, 1.1, 1.6, 1.0, 0.6),
            ("box", 4.2, -7.8, 0.9, 1.4, 0.7, -0.22),
            ("cyl", 10.4, 2.2, 0.42, 1.1),
            ("cyl", -6.2, -1.8, 0.35, 0.9),
            ("cyl", 1.6, 8.4, 0.48, 1.2),
            ("sph", 9.2, -8.6, 0.45),
            ("sph", -3.8, 9.6, 0.38),
            ("sph", 12.4, 8.2, 0.32),
            ("box", -11.2, 3.4, 1.4, 0.8, 0.8, 0.18),
            ("cyl", 8.6, -2.4, 0.3, 0.7),
        ],
        3: [
            ("box", -11.4, 1.8, 1.2, 1.8, 0.9, 0.25),
            ("box", 11.2, -6.4, 1.5, 1.1, 0.8, -0.5),
            ("box", 2.6, 10.2, 1.1, 1.3, 0.85, 0.4),
            ("box", -4.8, -8.6, 1.6, 0.9, 0.7, -0.15),
            ("cyl", 8.4, 7.2, 0.4, 1.0),
            ("cyl", -9.6, 8.8, 0.36, 0.85),
            ("cyl", 5.2, -10.4, 0.45, 1.15),
            ("sph", -2.2, 4.6, 0.42),
            ("sph", 10.8, 1.4, 0.34),
            ("sph", -12.2, -4.2, 0.38),
            ("box", 0.8, -3.2, 0.8, 1.2, 0.75, 0.7),
            ("cyl", -7.4, -2.6, 0.28, 0.7),
        ],
        4: [
            ("box", -10.2, 5.4, 0.9, 1.3, 0.8, 0.3),
            ("box", -2.6, -6.8, 1.4, 1.0, 0.7, -0.45),
            ("box", 5.2, 8.2, 1.6, 1.2, 0.9, 0.2),
            ("box", 11.4, -0.6, 1.1, 1.4, 0.85, -0.33),
            ("box", 8.8, -8.4, 1.3, 0.8, 0.75, 0.55),
            ("cyl", -6.8, 8.6, 0.38, 1.0),
            ("cyl", 2.4, -8.8, 0.42, 0.9),
            ("cyl", -12.6, -3.4, 0.33, 0.8),
            ("sph", 0.6, 3.2, 0.4),
            ("sph", 13.2, 6.4, 0.36),
            ("sph", -8.4, 0.8, 0.3),
            ("box", 14.2, 3.6, 1.0, 1.1, 0.7, 0.12),
            ("cyl", -4.2, 6.2, 0.26, 0.65),
        ],
        5: [
            ("box", -10.6, 4.8, 2.2, 0.9, 0.85, 0.18),
            ("box", -3.4, -5.6, 0.9, 2.4, 0.8, -0.28),
            ("box", 6.2, 7.4, 2.1, 0.85, 0.9, 0.4),
            ("box", 11.6, -7.2, 1.3, 1.3, 0.8, -0.5),
            ("cyl", -7.8, 8.8, 0.4, 1.05),
            ("cyl", 3.6, -8.4, 0.36, 0.85),
            ("cyl", 12.8, 4.2, 0.44, 1.1),
            ("sph", -1.2, 6.6, 0.38),
            ("sph", 8.4, 0.8, 0.42),
            ("sph", -12.2, -6.4, 0.34),
            ("box", 1.8, 10.6, 1.4, 1.0, 0.75, 0.22),
            ("cyl", -5.6, 1.4, 0.3, 0.7),
            ("sph", 9.6, 9.2, 0.3),
        ],
    }
    for i, item in enumerate(obstacles[index]):
        kind, x, y = item[0], item[1], item[2]
        ground = float(profile(np.asarray(x), np.asarray(y)))
        if kind == "box":
            sx, sy, sz, yaw = item[3:]
            box(link, f"rock_{i}", x, y, ground + sz / 2, sx, sy, sz, "obstacle", yaw=yaw,
                footprints=marks, level="lower")
        elif kind == "cyl":
            radius, height = item[3], item[4]
            cylinder(link, f"spire_{i}", x, y, ground + height / 2, radius, height, "accent",
                     footprints=marks, level="lower")
        else:
            radius = item[3]
            sphere(link, f"boulder_{i}", x, y, ground + radius, radius, "prop",
                   footprints=marks, level="lower")
    triangles = np.concatenate([surface, *primitive_triangles(link)])
    write_stl(package / "3DModel/terrain" / f"terrain_{index}.stl", triangles)
    for collision in list(link.findall("collision")):
        link.remove(collision)
    add_shape(link, "terrain_mesh", "mesh",
              mesh_uri("terrain", f"terrain_{index}.stl"), color="ground")
    link.find("visual[@name='terrain_mesh_visual']/geometry/mesh/uri").text = (
        mesh_uri("terrain", "surfaces", surface_path.name))
    goals = [(0, 0), (size[0] / 2 - 4, 0), (0, size[1] / 2 - 4),
             (size[0] / 2 - 4, size[1] / 2 - 4), (size[0] / 2 - 4, -size[1] / 2 + 4)]
    interior = (-size[0] / 2 + 0.5, size[0] / 2 - 0.5, -size[1] / 2 + 0.5, size[1] / 2 - 0.5)
    assert_connected(f"terrain_{index}", spawn[:2], goals, marks, interior, "lower")
    return root, size, spawn


def terrain_1_map(package):
    """Keep the original terrain_with_wall mesh, match the outdoor palette, add rocks."""
    size = (30.7, 30.7)
    spawn = (3.0, 9.0, .08, math.pi)
    mesh_path = package / "3DModel/terrain/terrain_1.stl"
    base_path = package / "3DModel/terrain/surfaces/terrain_1_base.stl"
    if not base_path.is_file():
        base_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(mesh_path, base_path)
    base = read_binary_stl(base_path)
    surface, walls = split_terrain_visuals(base)
    surface_path = package / "3DModel/terrain/surfaces/terrain_1_surface.stl"
    walls_path = package / "3DModel/terrain/surfaces/terrain_1_walls.stl"
    write_stl(surface_path, surface)
    write_stl(walls_path, walls)
    root, link = new_world("terrain_1", size)
    marks = []
    for x, y, sx, sy in ((-15.45, 0, 0.3, 31.0), (15.45, 0, 0.3, 31.0),
                         (0, -15.45, 31.0, 0.3), (0, 15.35, 31.0, 0.3)):
        track(marks, "all", x, y, sx, sy, 0)
    obstacles = [
        ("box", -8.6, -4.4, 1.4, 1.0, 0.85, 0.32),
        ("box", 6.4, -7.8, 1.2, 1.5, 0.8, -0.42),
        ("box", -5.2, 4.6, 1.55, 0.9, 0.9, 0.18),
        ("box", 9.6, 3.8, 1.1, 1.3, 0.75, 0.55),
        ("box", -11.4, 7.2, 1.3, 0.85, 0.8, -0.22),
        ("box", 8.8, 11.4, 1.0, 1.15, 0.7, 0.28),
        ("cyl", 1.2, -6.8, 0.4, 1.05),
        ("cyl", -9.8, 1.2, 0.36, 0.9),
        ("cyl", 11.4, -2.4, 0.42, 1.1),
        ("cyl", -2.6, -10.6, 0.32, 0.8),
        ("sph", 7.6, 7.2, 0.38),
        ("sph", -6.4, -9.4, 0.42),
        ("sph", 4.2, -3.4, 0.34),
        ("sph", -12.2, -6.2, 0.36),
    ]
    for i, item in enumerate(obstacles):
        kind, x, y = item[0], item[1], item[2]
        ground = mesh_support_z(base, x, y)
        if kind == "box":
            sx, sy, sz, yaw = item[3:]
            box(link, f"rock_{i}", x, y, ground + sz / 2, sx, sy, sz, "obstacle", yaw=yaw,
                footprints=marks, level="lower")
        elif kind == "cyl":
            radius, height = item[3], item[4]
            cylinder(link, f"spire_{i}", x, y, ground + height / 2, radius, height, "accent",
                     footprints=marks, level="lower")
        else:
            radius = item[3]
            sphere(link, f"boulder_{i}", x, y, ground + radius, radius, "prop",
                   footprints=marks, level="lower")
    write_stl(mesh_path, np.concatenate([base, *primitive_triangles(link)]))
    for collision in list(link.findall("collision")):
        link.remove(collision)
    add_shape(link, "terrain_mesh", "mesh", mesh_uri("terrain", "terrain_1.stl"), color="ground")
    link.find("visual[@name='terrain_mesh_visual']/geometry/mesh/uri").text = (
        mesh_uri("terrain", "surfaces", "terrain_1_surface.stl"))
    add_shape(link, "terrain_walls", "mesh", mesh_uri("terrain", "surfaces", "terrain_1_walls.stl"),
              color="wall", collide=False)
    goals = [(0, 0), (-10, 0), (10, -8), (0, -10), (-10, 6), (10, 8)]
    interior = (-14.8, 14.8, -14.8, 14.8)
    assert_connected("terrain_1", spawn[:2], goals, marks, interior, "lower")
    return root, size, spawn


def floor_surface_profile(index, level, xmin, xmax, sy, spawn, ramp_y):
    def profile(x, y):
        if index == 2 and level == "upper":
            z = .16 * np.sin((x - xmin) / 6.2) * np.cos(y / 9.0)
            z += .38 * np.exp(-((x + 10) ** 2 / 16 + (y - 5.5) ** 2 / 12))
        elif index == 3 and level == "lower":
            z = .30 * np.exp(-(y - 2.2) ** 2 / 18) * (.55 + .45 * np.cos(x / 7))
            z += .18 * np.sin(x / 3.8) * np.cos((y + 1.5) / 5.5)
        elif index == 5 and level == "lower":
            z = .16 * (1 + np.tanh((x + 5) / 2.6)) + .12 * (1 + np.tanh((x - 7) / 3.0))
            z += .08 * np.sin(y / 3.4)
            z *= 0.7
        else:
            z = .20 * np.sin((x + 0.6 * y) / 5.2) ** 2
            z += .36 * np.exp(-((x + 11) ** 2 / 14 + (y + 2) ** 2 / 11))
            z += .16 * np.cos(y / 4.4) * np.sin(x / 8)
        edge = smoothstep(np.minimum.reduce((x - xmin, xmax - x, sy / 2 - np.abs(y))) / 2)
        safe = smoothstep((np.hypot(x - spawn[0], y - spawn[1]) - 1.5) / 1.5)
        approach = 1 - smoothstep((y - (ramp_y - 4.0)) / 2.0)
        return np.maximum(z, 0) * edge * safe * approach

    return profile


def floor_map(index, package):
    sx, sy, upper_edge, upper_z, run, ramp_y, width = (
        (32, 24, 2, 2.4, 10, 7.5, 2.6) if index == 4 else (36, 24, 4, 3.0, 12, 8, 3.0))
    lower_rough, upper_rough = index in (3, 5), index in (2, 5)
    root, link = new_world(f"floor_{index}", (sx, sy))
    marks = []
    lower_spawn = (-sx / 2 + 3.5, -sy / 2 + 3.5, .08, 0)
    upper_spawn = (-6, -7, upper_z + .08, 0)
    for level, rough, xmin, xmax, height, spawn in (
            ("lower", lower_rough, -sx / 2, sx / 2, 0, lower_spawn),
            ("upper", upper_rough, -sx / 2, upper_edge, upper_z, upper_spawn)):
        if rough:
            path = package / "3DModel/floor/surfaces" / f"floor_{index}_{level}.stl"
            write_stl(path, heightfield((xmin, xmax, -sy / 2, sy / 2),
                                        floor_surface_profile(index, level, xmin, xmax, sy, spawn, ramp_y),
                                        base=height))
            add_shape(link, f"{level}_terrain", "mesh", mesh_uri("floor", "surfaces", path.name),
                      color="lower_terrain" if level == "lower" else "upper_terrain")
        else:
            box(link, f"{level}_floor", (xmin + xmax) / 2, 0, height - .12,
                xmax - xmin, sy, .24, "ground" if level == "lower" else "upper")

    boundary(link, sx, sy, height=2.1, footprints=marks)
    box(link, "upper_west_rail", -sx / 2 + .06, 0, upper_z + .4, .12, sy, .8, "accent")
    for name, y in (("north", sy / 2 - .06), ("south", -sy / 2 + .06)):
        box(link, f"upper_{name}_rail", (upper_edge - sx / 2) / 2, y,
            upper_z + .4, sx / 2 + upper_edge, .12, .8, "accent")
    for name, y0, y1 in (("south", -sy / 2, ramp_y - width / 2 - .12),
                        ("north", ramp_y + width / 2 + .12, sy / 2)):
        box(link, f"upper_east_{name}_rail", upper_edge - .06, (y0 + y1) / 2,
            upper_z + .4, .12, y1 - y0, .8, "accent")

    pitch = math.atan2(upper_z, run)
    ramp_length = math.hypot(upper_z, run)
    normal = np.array([math.sin(pitch), 0, math.cos(pitch)])
    midpoint = np.array([upper_edge + run / 2, ramp_y, upper_z / 2])
    center = midpoint - normal * .08
    box(link, "connecting_ramp", *center, ramp_length, width, .16, "ramp", pitch=pitch)
    for side in (-1, 1):
        rail = midpoint + normal * .23
        rail[1] += side * (width / 2 + .05)
        box(link, f"ramp_rail_{side}", *rail, ramp_length, .1, .46, "accent", pitch=pitch)
    for i, (x, y) in enumerate(((-sx / 2 + 2.2, -8.4), (-sx / 2 + 2.4, 8.6), (upper_edge - 2.2, -8.8))):
        cylinder(link, f"support_{i}", x, y, (upper_z - .25) / 2, .22, upper_z - .25, "structure",
                 footprints=marks)

    lower_plans = {
        2: [(-7.4, 1.6, .18, 8.2, 0.08), (1.8, -5.2, 7.4, .18, -0.12),
            (-2.6, 6.2, 5.6, .18, 0.16), (8.4, -1.8, .18, 6.4, -0.05)],
        3: [(-9.2, 3.4, 6.4, .18, 0.1), (1.2, -3.8, .18, 7.6, -0.07),
            (7.6, 4.2, 5.8, .18, 0.18), (-4.4, -6.6, 4.8, .18, -0.14)],
        4: [(-5.2, 1.2, 7.2, .18, 0.06), (-10.2, 4.1, .18, 5.4, -0.1),
            (5.4, -4.8, 4.6, .18, 0.2), (8.8, 3.6, .18, 6.0, -0.04)],
        5: [(-11.2, 1.4, .18, 8.6, 0.05), (-3.8, -4.2, 6.4, .18, -0.16),
            (8.6, -5.6, .18, 5.8, 0.12), (2.4, 5.8, 5.2, .18, 0.22)],
    }
    upper_plans = {
        2: [(-11.4, 2.2, .18, 7.4, 0.07), (-4.6, 3.4, 6.8, .18, -0.11),
            (-8.2, -4.8, 4.6, .18, 0.18)],
        3: [(-10.2, -2.2, 5.4, .18, 0.09), (-2.6, 2.4, .18, 7.8, -0.06),
            (-7.4, 6.2, 4.8, .18, 0.14)],
        4: [(-9.2, -3.6, .18, 8.2, 0.04), (-2.8, 2.2, 6.4, .18, -0.13),
            (-6.6, 6.8, 4.2, .18, 0.2)],
        5: [(-12.2, 1.6, 5.2, .18, 0.1), (-5.2, -2.4, .18, 7.2, -0.08),
            (-8.8, 6.4, 4.6, .18, 0.16)],
    }
    for level, plan, height in (("lower", lower_plans[index], 0), ("upper", upper_plans[index], upper_z)):
        for i, (x, y, bx, by, yaw) in enumerate(plan):
            box(link, f"{level}_partition_{i}", x, y, height + .9, bx, by, 1.8, "structure",
                yaw=yaw, footprints=marks, level=level)

    lower_props = {
        2: [(-12.4, 6.8, 1.1, 1.0, 0.25), (10.6, -7.2, 1.2, 0.9, -0.4),
            (-1.2, 8.6, 0.9, 1.2, 0.15), (6.2, 2.4, 1.0, 0.8, 0.5)],
        3: [(-12.8, -6.4, 1.15, 1.05, 0.2), (11.2, 6.6, 1.0, 1.2, -0.3),
            (-6.8, 8.2, 0.95, 0.85, 0.45), (4.4, -8.0, 1.2, 0.9, -0.18)],
        4: [(-11.6, -6.8, 1.1, 1.1, 0.12), (9.4, 7.2, 1.25, 0.9, -0.35),
            (-2.2, 8.4, 0.9, 1.15, 0.28), (3.6, -7.6, 1.05, 0.8, 0.4)],
        5: [(-13.2, 7.4, 1.2, 1.0, 0.22), (11.4, -7.8, 1.1, 1.15, -0.28),
            (-0.6, 8.8, 0.85, 1.05, 0.18), (6.8, 1.6, 1.0, 0.9, 0.55)],
    }
    for i, (x, y, px, py, yaw) in enumerate(lower_props[index]):
        box(link, f"lower_crate_{i}", x, y, .55, px, py, 1.1, "obstacle", yaw=yaw,
            footprints=marks, level="lower")
    for i, (x, y) in enumerate({
        2: [(-9.6, -6.2), (12.8, 3.4), (3.2, -8.4)],
        3: [(-11.4, 6.2), (13.2, -3.6), (0.8, 8.8)],
        4: [(-8.8, 7.6), (10.6, -6.4), (1.4, -8.2)],
        5: [(-10.2, -6.8), (13.0, 6.2), (4.6, -8.6)],
    }[index]):
        cylinder(link, f"lower_barrel_{i}", x, y, .42, 0.28, 0.84, "accent",
                 footprints=marks, level="lower")
    upper_crates = {
        2: [(-13.4, -3.2, 1.1, 1.4, 0.2), (-1.8, 5.6, 1.0, 0.9, -0.35)],
        3: [(-13.0, 4.4, 1.2, 1.1, 0.15), (-4.8, -6.2, 0.95, 1.3, 0.4)],
        4: [(-12.2, -5.4, 1.15, 1.2, -0.22), (-3.2, 7.2, 1.0, 1.05, 0.3)],
        5: [(-13.6, 3.8, 1.05, 1.35, 0.18), (-2.4, -5.8, 1.1, 0.95, -0.28)],
    }
    for i, (x, y, px, py, yaw) in enumerate(upper_crates[index]):
        box(link, f"upper_crate_{i}", x, y, upper_z + .55, px, py, 1.1, "obstacle", yaw=yaw,
            footprints=marks, level="upper")
    for i, (x, y) in enumerate({
        2: [(-9.8, 7.4), (-1.2, -6.6)],
        3: [(-12.4, -6.8), (-3.6, 7.6)],
        4: [(-10.6, 1.8), (-4.4, -7.2)],
        5: [(-11.8, 7.2), (-0.8, 4.4)],
    }[index]):
        cylinder(link, f"upper_barrel_{i}", x, y, upper_z + .4, 0.26, 0.8, "prop",
                 footprints=marks, level="upper")
    sphere(link, "upper_stone", -7.6, 0.6, upper_z + 0.22, 0.22, "prop",
           footprints=marks, level="upper")

    ramp_start = [upper_edge + run - 1.0, ramp_y, upper_z / run + .08, math.pi]
    lower_goals = [(0, 0), (sx / 2 - 4, -sy / 2 + 3), (upper_edge + run - 1.5, ramp_y),
                   (0, sy / 2 - 3), (sx / 2 - 4, 0)]
    upper_goals = [(-sx / 2 + 4, 0), (upper_edge - 1.5, ramp_y), (-sx / 2 + 4, sy / 2 - 3),
                   (upper_edge - 2, -sy / 2 + 3)]
    lower_interior = (-sx / 2 + 0.5, sx / 2 - 0.5, -sy / 2 + 0.5, sy / 2 - 0.5)
    upper_interior = (-sx / 2 + 0.5, upper_edge - 0.4, -sy / 2 + 0.5, sy / 2 - 0.5)
    assert_connected(f"floor_{index}_lower", lower_spawn[:2], lower_goals, marks, lower_interior, "lower")
    assert_connected(f"floor_{index}_upper", upper_spawn[:2], upper_goals, marks, upper_interior, "upper")
    return root, (sx, sy), lower_spawn, upper_spawn, ["terrain" if lower_rough else "flat",
                                                     "terrain" if upper_rough else "flat"], ramp_start


def set_rgba(material, rgba):
    for child in list(material):
        material.remove(child)
    text(material, "ambient", rgba)
    text(material, "diffuse", rgba)


def normalize_baselines(package):
    for category, name in (("flat", "flat_1"), ("floor", "floor_1")):
        path = package / "worlds" / category / f"{name}.world"
        root = ET.parse(path).getroot()
        world = root.find("world"); world.set("name", name)
        PALETTE.update(PALETTES[category])
        scene = world.find("scene")
        if scene is None:
            scene = ET.SubElement(world, "scene")
        for tag in ("ambient", "background"):
            node = scene.find(tag)
            if node is None:
                node = ET.SubElement(scene, tag)
            node.text = PALETTES[category][tag]
        for model in world.findall("model"):
            model.set("name", name + "_structure")
        for material in world.findall(".//material"):
            script_name = material.findtext("script/name")
            if script_name in BASELINE_MATERIALS:
                key = BASELINE_MATERIALS[script_name]
                rgba = PALETTES[category].get(key, PALETTES[category]["prop"])
                set_rgba(material, rgba)
        save_xml(path, root)


def generate(package, spacing):
    normalize_baselines(package)
    descriptions = {
        "flat_1": "原室内平地：房间、隔墙及家具",
        "flat_2": "仓储：错开货架、桶装货物和斜放托盘",
        "flat_3": "办公室：不对称隔间、桌椅和绿植",
        "flat_4": "广场：错落立柱、花坛、座椅和石墩",
        "flat_5": "迷宫：不等长隔墙、侧向回路和散置障碍",
        "terrain_1": "原带墙起伏地形，散落岩块与石柱，室外配色",
        "terrain_2": "分散丘陵：缓丘间散布岩块与石柱",
        "terrain_3": "蜿蜒谷地：谷旁错落岩块，中央通道可通行",
        "terrain_4": "短波起伏：缓包之间散置石墩和短墙",
        "terrain_5": "连续台地：阶地旁散布岩柱与石块",
        "floor_1": "原双层室内：两层平坦、折返坡道",
        "floor_2": "下层平坦、上层西侧缓丘，直线坡道",
        "floor_3": "下层中央浅脊、上层平坦，直线坡道",
        "floor_4": "两层平坦：不同隔墙与散置障碍",
        "floor_5": "下层缓台地、上层斜向起伏，直线坡道",
    }
    catalog = {"schema_version": 1, "frame_id": "map", "units": "m",
               "pcd_sampling": {"terrain_spacing": spacing, "wall_spacing": spacing}, "maps": {}}
    for category in ("flat", "terrain", "floor"):
        for index in range(1, 6):
            name = f"{category}_{index}"
            upper_spawn = None; floor_types = [category]; ramp_start = None
            if index == 1 and category == "terrain":
                root, size, spawn = terrain_1_map(package)
                save_xml(package / "worlds" / category / f"{name}.world", root)
            elif index == 1:
                size, spawn = {"flat": ((12, 9), (0, 0, .08, 0)),
                               "floor": ((13, 11), (0, 0, .08, 0))}[category]
                if category == "floor":
                    upper_spawn = (-1, -4, 1.08, 0); floor_types = ["flat", "flat"]
                    ramp_start = [4.25, -3.8, .13, math.pi / 2]
            elif category == "flat":
                root, size = flat_map(index)
                spawn = (-size[0] / 2 + 1.0, -size[1] / 2 + 1.0, .08, math.pi / 2)
                if index == 4:
                    spawn = (-7, 0, .08, 0)
                save_xml(package / "worlds" / category / f"{name}.world", root)
            elif category == "terrain":
                root, size, spawn = terrain_map(index, package)
                save_xml(package / "worlds" / category / f"{name}.world", root)
            else:
                root, size, spawn, upper_spawn, floor_types, ramp_start = floor_map(index, package)
                save_xml(package / "worlds" / category / f"{name}.world", root)
            world_path = package / "worlds" / category / f"{name}.world"
            mesh_path = package / "3DModel" / category / f"{name}.stl"
            if category != "terrain":
                write_stl(mesh_path, world_triangles(world_path, package))
            triangles = read_binary_stl(mesh_path)
            points, _stats = sample_navigation_surfaces(triangles, spacing, spacing)
            cloud_path = package / "pcd" / category / f"{name}.pcd"
            write_binary_pcd(cloud_path, points)
            interior_points, _interior_stats = sample_navigation_surfaces(
                triangles, spacing, spacing, interior_from_size(size))
            write_binary_pcd(package / "pcd_modified" / category / f"{name}.pcd", interior_points)
            record = {"category": category, "description": descriptions[name],
                      "world": f"worlds/{category}/{name}.world", "mesh": f"3DModel/{category}/{name}.stl",
                      "pcd": f"pcd/{category}/{name}.pcd", "size_xy": list(size),
                      "floor_types": floor_types, "spawn": [float(v) for v in spawn],
                      "points": len(points), "triangles": len(triangles),
                      "bounds_min": points.min(axis=0).astype(float).tolist(),
                      "bounds_max": points.max(axis=0).astype(float).tolist()}
            if upper_spawn is not None:
                record["upper_spawn"] = [float(v) for v in upper_spawn]
            if ramp_start is not None:
                record["ramp_start"] = [float(v) for v in ramp_start]
                record["ramp_waypoints"] = ([[4.25, .65], [2.75, .65], [2.75, -4.8], [.7, -4.8]]
                                            if index == 1 else [[1.2 if index == 4 else 3.2,
                                                                 7.5 if index == 4 else 8]])
            if name == "terrain_1":
                record["clearing_spawn"] = [3., 9., .3, math.pi]
            if name == "flat_4":
                record["clearing_spawn"] = [-6., -9., .1, 0.]
            catalog["maps"][name] = record
            print(f"{name}: {len(triangles)} triangles, {len(points)} points", flush=True)
    (package / "config/maps.yaml").write_text(yaml.safe_dump(catalog, allow_unicode=True, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, default=PACKAGE_DIR)
    parser.add_argument("--spacing", type=float, default=.10)
    args = parser.parse_args()
    if not 0 < args.spacing <= .2:
        parser.error("spacing must be positive and no larger than 0.2 m")
    generate(args.package_dir.resolve(), args.spacing)


if __name__ == "__main__":
    main()

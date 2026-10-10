#!/usr/bin/env python3
"""Export static Gazebo collision geometry in world coordinates to binary STL.

Boxes, cylinders, spheres, STL and the bundled COLLADA furniture are supported.
The exporter applies model, link, collision and include poses in that order.
"""

from __future__ import annotations

import argparse
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from stl_to_navigation_pcd import STL_DTYPE, read_binary_stl


def pose_matrix(text: str | None) -> np.ndarray:
    xyzrpy = [float(v) for v in (text or "0 0 0 0 0 0").split()]
    if len(xyzrpy) != 6:
        raise ValueError(f"Expected an xyz/rpy pose, got {text!r}")
    x, y, z, roll, pitch, yaw = xyzrpy
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                            math.cos(pitch), math.sin(pitch),
                            math.cos(yaw), math.sin(yaw))
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = (x, y, z)
    return result


def transform(triangles: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return triangles @ matrix[:3, :3].T + matrix[:3, 3]


def box_triangles(size) -> np.ndarray:
    vertices = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
    ], dtype=float) * np.asarray(size) / 2.0
    faces = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
             [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
             [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]]
    return vertices[faces]


def cylinder_triangles(radius: float, length: float, segments=64) -> np.ndarray:
    angle = np.arange(segments + 1) * (2 * math.pi / segments)
    bottom = np.column_stack((radius * np.cos(angle), radius * np.sin(angle),
                              np.full(len(angle), -length / 2)))
    top = bottom.copy()
    top[:, 2] = length / 2
    triangles = []
    for i in range(segments):
        triangles.extend([
            [bottom[i], bottom[i + 1], top[i + 1]],
            [bottom[i], top[i + 1], top[i]],
            [[0, 0, -length / 2], bottom[i + 1], bottom[i]],
            [[0, 0, length / 2], top[i], top[i + 1]],
        ])
    return np.asarray(triangles, dtype=float)


def sphere_triangles(radius: float, rings=12, segments=20) -> np.ndarray:
    phis = np.linspace(0, math.pi, rings + 1)
    thetas = np.linspace(0, 2 * math.pi, segments, endpoint=False)
    triangles = []

    def vertex(ring, segment):
        phi, theta = phis[ring], thetas[segment % segments]
        return [radius * math.sin(phi) * math.cos(theta),
                radius * math.sin(phi) * math.sin(theta),
                radius * math.cos(phi)]

    pole_north, pole_south = [0, 0, radius], [0, 0, -radius]
    for segment in range(segments):
        triangles.append([pole_north, vertex(1, segment), vertex(1, segment + 1)])
        triangles.append([pole_south, vertex(rings - 1, segment + 1), vertex(rings - 1, segment)])
    for ring in range(1, rings - 1):
        for segment in range(segments):
            a, b = vertex(ring, segment), vertex(ring, segment + 1)
            c, d = vertex(ring + 1, segment), vertex(ring + 1, segment + 1)
            triangles.extend([[a, c, d], [a, d, b]])
    return np.asarray(triangles, dtype=float)


def write_stl(path: Path, triangles: np.ndarray) -> None:
    triangles = np.asarray(triangles, dtype=float)
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(cross, axis=1)
    valid = lengths > 1e-10
    triangles, cross, lengths = triangles[valid], cross[valid], lengths[valid]
    records = np.zeros(len(triangles), dtype=STL_DTYPE)
    records["normal"] = cross / lengths[:, None]
    records["vertices"] = triangles
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(b"MultiMapNav static world collision geometry".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(records)))
        stream.write(records.tobytes())


def resolve_uri(uri: str, package_dir: Path, relative_to: Path) -> Path:
    package_dir = Path(package_dir)
    relative_to = Path(relative_to)
    candidates = []
    if uri.startswith("model://"):
        suffix = uri[len("model://"):]
        if suffix.startswith("multimap_nav_simulation/"):
            candidates.append(package_dir / suffix.split("/", 1)[1])
        else:
            candidates.append(package_dir / "models" / suffix)
            candidates.append(package_dir / suffix)
    elif uri.startswith("file://"):
        path = Path(uri[len("file://"):])
        candidates.extend([path] if path.is_absolute() else [package_dir / path, relative_to / path])
    else:
        path = Path(uri)
        candidates.extend([path] if path.is_absolute() else [package_dir / path, relative_to / path])
    for candidate in candidates:
        if candidate.exists():
            return candidate
    tried = ", ".join(str(c) for c in candidates)
    raise FileNotFoundError(f"Unresolved Gazebo resource: {uri} ({tried})")


def read_collada(path: Path) -> np.ndarray:
    """Read COLLADA triangle positions, including asset units and scene transforms."""
    root = ET.parse(path).getroot()
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    unit = root.find("c:asset/c:unit", ns)
    unit_scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    geometries = {}
    for geom in root.findall("c:library_geometries/c:geometry", ns):
        mesh = geom.find("c:mesh", ns)
        sources = {}
        for source in mesh.findall("c:source", ns):
            accessor = source.find("c:technique_common/c:accessor", ns)
            stride = int(accessor.get("stride", "3"))
            values = np.fromstring(source.findtext("c:float_array", "", ns), sep=" ")
            sources[source.get("id")] = values.reshape(-1, stride)[:, :3]
        vertices = {v.get("id"): v.find("c:input", ns).get("source")[1:]
                    for v in mesh.findall("c:vertices", ns)}
        chunks = []
        for primitive in mesh.findall("c:triangles", ns):
            inputs = primitive.findall("c:input", ns)
            stride = 1 + max(int(item.get("offset", "0")) for item in inputs)
            item = next(item for item in inputs if item.get("semantic") == "VERTEX")
            positions = sources[vertices[item.get("source")[1:]]]
            indices = np.fromstring(primitive.findtext("c:p", "", ns), sep=" ", dtype=int)
            indices = indices.reshape(-1, stride)[:, int(item.get("offset", "0"))]
            chunks.append(positions[indices].reshape(-1, 3, 3))
        if not chunks:
            raise ValueError(f"COLLADA mesh has no triangles: {path}")
        geometries[geom.get("id")] = np.concatenate(chunks)

    chunks = []

    def visit(node, parent):
        matrix = parent.copy()
        for child in node:
            kind = child.tag.rsplit("}", 1)[-1]
            local = np.eye(4)
            if kind == "matrix":
                local = np.fromstring(child.text, sep=" ").reshape(4, 4).T
            elif kind == "translate":
                local[:3, 3] = np.fromstring(child.text, sep=" ")
            elif kind == "scale":
                local[:3, :3] = np.diag(np.fromstring(child.text, sep=" "))
            elif kind == "rotate":
                x, y, z, degrees = np.fromstring(child.text, sep=" ")
                axis = np.array([x, y, z]); axis /= np.linalg.norm(axis)
                angle = math.radians(degrees)
                skew = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                                 [-axis[1], axis[0], 0]])
                local[:3, :3] = (np.eye(3) * math.cos(angle)
                                 + (1 - math.cos(angle)) * np.outer(axis, axis)
                                 + math.sin(angle) * skew)
            else:
                continue
            matrix = matrix @ local
        for instance in node.findall("c:instance_geometry", ns):
            chunks.append(transform(geometries[instance.get("url")[1:]], matrix))
        for child in node.findall("c:node", ns):
            visit(child, matrix)

    scene = root.find("c:library_visual_scenes/c:visual_scene", ns)
    if scene is not None:
        for node in scene.findall("c:node", ns):
            visit(node, np.eye(4))
    if not chunks:
        chunks = list(geometries.values())
    result = np.concatenate(chunks) * unit_scale
    up_axis = root.findtext("c:asset/c:up_axis", "Y_UP", ns)
    if up_axis == "Y_UP":
        result = result @ pose_matrix("0 0 0 1.5707963267948966 0 0")[:3, :3].T
    return result


def world_triangles(world_path: Path, package_dir: Path) -> np.ndarray:
    chunks = []

    def visit(parent, matrix, base):
        for model in parent.findall("model"):
            # Path-driven actors are perceived at runtime and must never be
            # baked into a static navigation mesh at their initial position.
            if any(p.get("filename") == "libmultimap_obstacle_motion.so" for p in model.findall("plugin")):
                continue
            model_matrix = matrix @ pose_matrix(model.findtext("pose"))
            for link in model.findall("link"):
                link_matrix = model_matrix @ pose_matrix(link.findtext("pose"))
                for collision in link.findall("collision"):
                    collision_matrix = link_matrix @ pose_matrix(collision.findtext("pose"))
                    geom = collision.find("geometry")
                    if geom.find("box") is not None:
                        triangles = box_triangles(np.fromstring(geom.findtext("box/size"), sep=" "))
                    elif geom.find("cylinder") is not None:
                        triangles = cylinder_triangles(float(geom.findtext("cylinder/radius")),
                                                       float(geom.findtext("cylinder/length")))
                    elif geom.find("sphere") is not None:
                        triangles = sphere_triangles(float(geom.findtext("sphere/radius")))
                    elif geom.find("mesh") is not None:
                        path = resolve_uri(geom.findtext("mesh/uri"), package_dir, base)
                        triangles = read_collada(path) if path.suffix.lower() == ".dae" else read_binary_stl(path)
                        triangles *= np.fromstring(geom.findtext("mesh/scale", "1 1 1"), sep=" ")
                    elif geom.find("plane") is not None:
                        # The baseline worlds also include Gazebo's infinite ground plane.
                        # Their finite floor boxes define the navigation map extent.
                        continue
                    else:
                        raise ValueError(f"Unsupported collision geometry in {world_path}")
                    chunks.append(transform(triangles, collision_matrix))
            visit(model, model_matrix, base)
        for include in parent.findall("include"):
            model_path = resolve_uri(include.findtext("uri"), package_dir, base)
            if model_path.is_dir():
                config = ET.parse(model_path / "model.config").getroot()
                model_path = model_path / config.findtext("sdf")
            visit(ET.parse(model_path).getroot(), matrix @ pose_matrix(include.findtext("pose")), model_path.parent)

    visit(ET.parse(world_path).getroot().find("world"), np.eye(4), world_path.parent)
    if not chunks:
        raise ValueError(f"No finite collision geometry in {world_path}")
    return np.concatenate(chunks)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("world", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    triangles = world_triangles(args.world, args.package_dir)
    write_stl(args.output, triangles)
    print(f"{args.output}: {len(triangles)} triangles")


if __name__ == "__main__":
    main()

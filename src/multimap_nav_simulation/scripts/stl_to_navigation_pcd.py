#!/usr/bin/env python3
"""Generate a deterministic navigation PCD from a binary STL mesh.

The terrain surface is sampled densely enough for a 0.1 m GridMap. Large,
skinny wall faces are sampled by area so they do not consume most of the point
budget, and downward-facing faces are omitted because they are not traversable.

Passing ``interior`` keeps only the surfaces a robot inside the map can drive
on or see: floors, ramps, terrain, interior wall faces and obstacles. Wall and
rail tops, the outer face of the perimeter, and the vertical skirt around a
terrain volume are left out.
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path

import numpy as np
import json


STL_DTYPE = np.dtype(
    [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
)


def read_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * STL_DTYPE.itemsize
    if len(data) != expected_size:
        raise ValueError(
            f"Expected a binary STL of {expected_size} bytes, got {len(data)} bytes"
        )
    return np.frombuffer(data, dtype=STL_DTYPE, offset=84, count=triangle_count)[
        "vertices"
    ].astype(np.float64)


def triangular_lattice(triangle: np.ndarray, spacing: float) -> np.ndarray:
    edges = np.roll(triangle, -1, axis=0) - triangle
    subdivision = max(1, int(math.ceil(np.linalg.norm(edges, axis=1).max() / spacing)))
    rows = []
    for i in range(subdivision + 1):
        a = i / subdivision
        b = np.arange(subdivision + 1 - i, dtype=float) / subdivision
        rows.append(triangle[0] + a * (triangle[1] - triangle[0])
                    + b[:, None] * (triangle[2] - triangle[0]))
    return np.concatenate(rows).astype(np.float32)


def area_samples(triangle: np.ndarray, spacing: float) -> np.ndarray:
    cross = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
    area = 0.5 * float(np.linalg.norm(cross))
    count = max(3, int(math.ceil(area / (spacing * spacing))))
    # Deterministic low-discrepancy sequence, folded into a triangle.
    index = np.arange(count, dtype=np.float64) + 0.5
    u = index / count
    v = np.mod(index * 0.6180339887498949, 1.0)
    reflected = u + v > 1.0
    u[reflected] = 1.0 - u[reflected]
    v[reflected] = 1.0 - v[reflected]
    result = (
        triangle[0]
        + u[:, None] * (triangle[1] - triangle[0])
        + v[:, None] * (triangle[2] - triangle[0])
    )
    return result.astype(np.float32)


def _upward_altitude(triangle: np.ndarray) -> tuple[float, float]:
    """Return the horizontal altitude to the longest edge, and that edge length."""
    span = triangle.max(axis=0) - triangle.min(axis=0)
    longest = float(max(span[0], span[1], 1.0e-12))
    horizontal_area = 0.5 * abs(float(np.cross(triangle[1, :2] - triangle[0, :2],
                                                triangle[2, :2] - triangle[0, :2])))
    return 2.0 * horizontal_area / longest, longest


def is_thin_cap(triangle: np.ndarray, normal_z: float) -> bool:
    """Wall and rail tops: narrow, long, and above the ground."""
    if normal_z <= 0.2 or float(triangle[:, 2].mean()) <= 0.30:
        return False
    altitude, longest = _upward_altitude(triangle)
    return altitude < 0.40 and longest > 0.80


def is_exterior_vertical(
    triangle: np.ndarray,
    normal: np.ndarray,
    interior: tuple[float, float, float, float],
) -> bool:
    """Outer perimeter faces and terrain skirts, which are not visible from inside.

    A face that sits just outside the floor slab is kept when it looks back into
    the map. floor_1's south wall is mounted outside the slab, and its inner
    face is still visible to the robot.
    """
    xmin, xmax, ymin, ymax = interior
    xs, ys = triangle[:, 0], triangle[:, 1]
    if np.all(xs <= xmin + 0.04) and normal[0] < -0.5:
        return True
    if np.all(xs >= xmax - 0.04) and normal[0] > 0.5:
        return True
    if np.all(ys <= ymin + 0.04) and normal[1] < -0.5:
        return True
    if np.all(ys >= ymax - 0.04) and normal[1] > 0.5:
        return True
    centroid = triangle.mean(axis=0)
    outside = (centroid[0] < xmin - 0.02 or centroid[0] > xmax + 0.02
               or centroid[1] < ymin - 0.02 or centroid[1] > ymax + 0.02)
    if not outside:
        return False
    center = np.array([(xmin + xmax) * 0.5, (ymin + ymax) * 0.5])
    toward = center - centroid[:2]
    return float(np.dot(normal[:2], toward)) <= 0.0


def sample_navigation_surfaces(
    triangles: np.ndarray,
    terrain_spacing: float,
    wall_spacing: float,
    interior: tuple[float, float, float, float] | None = None,
) -> tuple[np.ndarray, dict[str, int]]:
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    length = np.linalg.norm(cross, axis=1)
    normals = cross / np.maximum(length, 1.0e-12)[:, None]
    normal_z = normals[:, 2]
    chunks: list[np.ndarray] = []
    stats = {
        "terrain_triangles": 0,
        "wall_triangles": 0,
        "skipped_downward": 0,
        "skipped_thin_cap": 0,
        "skipped_exterior": 0,
    }
    for triangle, normal, nz, norm in zip(triangles, normals, normal_z, length):
        if norm < 1.0e-10:
            continue
        if nz < -0.05:
            stats["skipped_downward"] += 1
            continue
        if interior is not None and is_thin_cap(triangle, float(nz)):
            stats["skipped_thin_cap"] += 1
            continue
        if interior is not None and nz <= 0.2 and is_exterior_vertical(triangle, normal, interior):
            stats["skipped_exterior"] += 1
            continue
        # Large primitive floor/ramp faces need the same dense coverage as
        # terrain triangles; otherwise 0.1 m navigation cells contain holes.
        if nz > 0.2:
            chunks.append(triangular_lattice(triangle, terrain_spacing))
            stats["terrain_triangles"] += 1
        else:
            chunks.append(area_samples(triangle, wall_spacing))
            stats["wall_triangles"] += 1

    if not chunks:
        raise ValueError("Mesh contains no nondegenerate navigation surfaces")
    points = np.concatenate(chunks, axis=0)
    # Remove exact duplicates along shared triangle edges while preserving float32.
    points = np.unique(points, axis=0)
    return points, stats


def interior_from_size(size_xy: tuple[float, float]) -> tuple[float, float, float, float]:
    half_x, half_y = size_xy[0] / 2.0, size_xy[1] / 2.0
    return (-half_x, half_x, -half_y, half_y)


def export_modified_suite(
    package: Path,
    output_dir: Path,
    terrain_spacing: float,
    wall_spacing: float,
) -> None:
    """Write interior clouds for every map in worlds/catalog.json."""
    catalog = json.loads((package / "worlds" / "catalog.json").read_text())
    for name, record in catalog["maps"].items():
        mesh_path = package / record["mesh"]
        interior = interior_from_size(record["size_xy"])
        points, stats = sample_navigation_surfaces(
            read_binary_stl(mesh_path), terrain_spacing, wall_spacing, interior
        )
        cloud_path = output_dir / record["category"] / f"{name}.pcd"
        write_binary_pcd(cloud_path, points)
        print(f"{name}: {len(points)} points -> {cloud_path.relative_to(package)}")
        print(" ".join(f"{key}={value}" for key, value in stats.items()), flush=True)


def write_binary_pcd(path: Path, points: np.ndarray) -> None:
    points = np.ascontiguousarray(points, dtype="<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(points.tobytes(order="C"))


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("input", nargs="?", type=Path)
    parser.add_argument("output", nargs="?", type=Path)
    parser.add_argument("--terrain-spacing", type=float, default=0.07)
    parser.add_argument("--wall-spacing", type=float, default=0.10)
    parser.add_argument("--interior", type=float, nargs=4, metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                        help="Keep surfaces inside this rectangle and drop wall tops")
    parser.add_argument("--suite", action="store_true",
                        help="Write an interior cloud for every map in worlds/catalog.json")
    parser.add_argument("--output-dir", type=Path, default=package / "pcd_modified")
    parser.add_argument("--package-dir", type=Path, default=package)
    args = parser.parse_args()
    if args.terrain_spacing <= 0.0 or args.wall_spacing <= 0.0:
        parser.error("sampling spacings must be positive")

    if args.suite:
        export_modified_suite(
            args.package_dir.resolve(), args.output_dir.resolve(),
            args.terrain_spacing, args.wall_spacing,
        )
        return
    if args.input is None or args.output is None:
        parser.error("input and output are required unless --suite is set")

    triangles = read_binary_stl(args.input)
    interior = tuple(args.interior) if args.interior is not None else None
    points, stats = sample_navigation_surfaces(
        triangles, args.terrain_spacing, args.wall_spacing, interior
    )
    write_binary_pcd(args.output, points)
    print(f"triangles={len(triangles)} points={len(points)}")
    print(" ".join(f"{key}={value}" for key, value in stats.items()))
    print(f"bounds_min={points.min(axis=0)} bounds_max={points.max(axis=0)}")


if __name__ == "__main__":
    main()

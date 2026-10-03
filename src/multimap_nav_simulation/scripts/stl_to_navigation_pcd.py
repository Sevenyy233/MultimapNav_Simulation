#!/usr/bin/env python3
"""Generate a deterministic navigation PCD from a binary STL mesh.

The terrain surface is sampled densely enough for a 0.1 m GridMap. Large,
skinny wall faces are sampled by area so they do not consume most of the point
budget, and downward-facing faces are omitted because they are not traversable.
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path

import numpy as np


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
    points = []
    for i in range(subdivision + 1):
        for j in range(subdivision + 1 - i):
            a = i / subdivision
            b = j / subdivision
            points.append(triangle[0] + a * (triangle[1] - triangle[0]) + b * (triangle[2] - triangle[0]))
    return np.asarray(points, dtype=np.float32)


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


def sample_navigation_surfaces(
    triangles: np.ndarray,
    terrain_spacing: float,
    wall_spacing: float,
) -> tuple[np.ndarray, dict[str, int]]:
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    length = np.linalg.norm(cross, axis=1)
    normal_z = cross[:, 2] / np.maximum(length, 1.0e-12)
    max_edge = np.max(
        np.stack(
            [
                np.linalg.norm(triangles[:, 1] - triangles[:, 0], axis=1),
                np.linalg.norm(triangles[:, 2] - triangles[:, 1], axis=1),
                np.linalg.norm(triangles[:, 0] - triangles[:, 2], axis=1),
            ],
            axis=1,
        ),
        axis=1,
    )

    chunks: list[np.ndarray] = []
    stats = {"terrain_triangles": 0, "wall_triangles": 0, "skipped_downward": 0}
    for triangle, nz, edge in zip(triangles, normal_z, max_edge):
        if nz < -0.05:
            stats["skipped_downward"] += 1
            continue
        if nz > 0.2 and edge < 1.0:
            chunks.append(triangular_lattice(triangle, terrain_spacing))
            stats["terrain_triangles"] += 1
        else:
            chunks.append(area_samples(triangle, wall_spacing))
            stats["wall_triangles"] += 1

    points = np.concatenate(chunks, axis=0)
    # Remove exact duplicates along shared triangle edges while preserving float32.
    points = np.unique(points, axis=0)
    return points, stats


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
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--terrain-spacing", type=float, default=0.07)
    parser.add_argument("--wall-spacing", type=float, default=0.10)
    args = parser.parse_args()
    if args.terrain_spacing <= 0.0 or args.wall_spacing <= 0.0:
        parser.error("sampling spacings must be positive")

    triangles = read_binary_stl(args.input)
    points, stats = sample_navigation_surfaces(
        triangles, args.terrain_spacing, args.wall_spacing
    )
    write_binary_pcd(args.output, points)
    print(f"triangles={len(triangles)} points={len(points)}")
    print(" ".join(f"{key}={value}" for key, value in stats.items()))
    print(f"bounds_min={points.min(axis=0)} bounds_max={points.max(axis=0)}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Regenerate mesh-free cuboid envelopes using the original CAD as an offline reference.

This optional authoring tool requires NumPy and the bundled binary STL reader.
The generated xacro contains only numeric box/cylinder dimensions. Launching
and simulating the robot never reads these reference meshes.
"""

import argparse
from itertools import permutations
import math
from pathlib import Path

import numpy as np

from stl_to_navigation_pcd import read_binary_stl


def fit(path, step=0.09):
    """Sample vertical CAD envelopes, then merge occupied cells into cuboids."""
    triangles = read_binary_stl(path)
    low, high = triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))
    counts = np.maximum(1, np.ceil((high-low)/step).astype(int))
    size = (high-low)/counts
    x = low[0] + (np.arange(counts[0])+0.5)*size[0]
    y = low[1] + (np.arange(counts[1])+0.5)*size[1]
    z = low[2] + (np.arange(counts[2])+0.5)*size[2]
    zlow = np.full(counts[:2], np.inf)
    zhigh = np.full(counts[:2], -np.inf)
    # Discard tiny CAD fasteners below the primitive sampling resolution.
    area = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                   triangles[:, 2]-triangles[:, 0]), axis=1)/2
    for triangle in triangles[area > 2e-5]:
        a, b, c = triangle
        determinant = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
        if abs(determinant) < 1e-12:
            continue
        starts = np.maximum(0, np.floor(
            (triangle.min(axis=0)[:2]-low[:2])/size[:2]-0.5).astype(int))
        ends = np.minimum(counts[:2], np.ceil(
            (triangle.max(axis=0)[:2]-low[:2])/size[:2]+0.5).astype(int)+1)
        for ix in range(starts[0], ends[0]):
            yy = y[starts[1]:ends[1]]
            u = ((b[1]-c[1])*(x[ix]-c[0]) + (c[0]-b[0])*(yy-c[1]))/determinant
            v = ((c[1]-a[1])*(x[ix]-c[0]) + (a[0]-c[0])*(yy-c[1]))/determinant
            valid = (u >= -1e-8) & (v >= -1e-8) & (u+v <= 1+1e-8)
            height = u*a[2] + v*b[2] + (1-u-v)*c[2]
            indices = np.arange(starts[1], ends[1])[valid]
            zlow[ix, indices] = np.minimum(zlow[ix, indices], height[valid])
            zhigh[ix, indices] = np.maximum(zhigh[ix, indices], height[valid])
    # Keep thin members, expanding the sampled envelope by at most half a cell.
    occupancy = ((z[None, None, :] >= zlow[:, :, None]-size[2]/2) &
                 (z[None, None, :] <= zhigh[:, :, None]+size[2]/2))
    if path.name == 'base_link.STL':
        # Track skins are authored separately as boxes and cylinders. Keep the
        # central chassis and original slew-bearing support from the CAD fit.
        occupancy &= ((np.abs(y[None, :, None]) < 1.10) | (z[None, None, :] > 1.12))

    def partition(order):
        remaining = np.transpose(occupancy, order).copy()
        boxes = []
        for a, b, c in np.argwhere(remaining):
            if not remaining[a, b, c]:
                continue
            cend = c+1
            while cend < remaining.shape[2] and remaining[a, b, cend]:
                cend += 1
            bend = b+1
            while bend < remaining.shape[1] and remaining[a, bend, c:cend].all():
                bend += 1
            aend = a+1
            while aend < remaining.shape[0] and remaining[aend, b:bend, c:cend].all():
                aend += 1
            remaining[a:aend, b:bend, c:cend] = False
            first = np.array([a, b, c])[np.argsort(order)]
            last = np.array([aend, bend, cend])[np.argsort(order)]
            boxes.append((low+(first+last)*size/2, (last-first)*size))
        return boxes

    boxes = min((partition(order) for order in permutations(range(3))), key=len)
    print(path.name, 'grid', counts, 'boxes', len(boxes), 'dimensions', high-low)
    return boxes


def generate(package, output):
    parts = {0: 'base_link.STL', 1: 'PingTai.STL', 2: 'DaBi.STL', 3: 'zhongbi.STL',
             4: 'xiaobi.STL', 5: 'R2.STL', 6: 'R1.STL', 7: 'candou.STL'}
    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
             '  <!-- Offline CAD envelope fit: arm grid <= 0.09 m, chassis <= 0.11 m.',
             '       Geometry stays in the original link frames. No runtime mesh. -->']
    for index, filename in parts.items():
        step = 0.11 if index == 0 else 0.065 if index == 7 else 0.09
        boxes = fit(package / 'meshes' / 'clearing_robot' / filename, step)
        lines.append(f'  <xacro:macro name="clearing_arm_geometry_{index}" params="collision:=true">')
        for center, size in boxes:
            xyz = ' '.join(f'{value:.6f}' for value in center)
            dimensions = ' '.join(f'{value:.6f}' for value in size)
            lines.append(f'    <xacro:box_shape size="{dimensions}" xyz="{xyz}" '
                         'material="body_grey" collision="${collision}"/>')
        if index == 4:
            # Original thin cross rods need analytic cylinders below the grid
            # spacing to retain their continuous appearance.
            rods = [((-.86, -1.035, 0), (1.52, -.72, 0), .055),
                    ((-1.36, -1.08, 0), (-.86, -1.035, 0), .025),
                    ((-.905, -.54, 0), (-.835, -1.53, 0), .025)]
            for start, end, radius in rods:
                a, b = np.array(start), np.array(end)
                center, direction = (a+b)/2, b-a
                xyz = ' '.join(f'{value:.6f}' for value in center)
                yaw = math.atan2(direction[1], direction[0])
                lines.append(f'    <xacro:cylinder_shape radius="{radius}" '
                             f'length="{np.linalg.norm(direction):.6f}" xyz="{xyz}" '
                             f'rpy="0 ${{pi/2}} {yaw:.6f}" '
                             'material="steel" collision="${collision}"/>')
        if index == 7:
            # A thin inclined sheet can fall between vertical sampling rays.
            # Preserve a continuous full-width scoop back and cheek edges
            # with inclined rectangular plates in the original Y-Z profile.
            back = [(.05, .15), (.35, .40), (.46, 1.10), (.42, 1.25)]
            plates = [(0.0, 2.5, a, b) for a, b in zip(back, back[1:])]
            plates += [(side, .075, (-.22, .15), (.35, 1.10))
                       for side in (-1.2125, 1.2125)]
            for center_x, width, start, end in plates:
                a, b = np.array(start), np.array(end)
                center, direction = (a+b)/2, b-a
                roll = math.atan2(-direction[0], direction[1])
                lines.append(f'    <xacro:box_shape size="{width} 0.04 '
                             f'{np.linalg.norm(direction):.6f}" '
                             f'xyz="{center_x} {center[0]:.6f} {center[1]:.6f}" '
                             f'rpy="{roll:.6f} 0 0" '
                             'material="body_grey" collision="${collision}"/>')
            # Filled cheek plates follow the CAD wedge cross section; their
            # narrow X thickness keeps the scoop interior open.
            heights = np.linspace(.15, 1.25, 12)
            for first, last in zip(heights, heights[1:]):
                center_z = (first+last)/2
                front_y = min(.35, -.22+(center_z-.15)*(.57/.95))
                rear_y = (.35+(center_z-.4)*(.13/.86) if center_z > .4
                          else .20+(center_z-.15)*(.15/.25))
                for side in (-1.2125, 1.2125):
                    lines.append(f'    <xacro:box_shape size="0.075 {rear_y-front_y:.6f} '
                                 f'{last-first:.6f}" '
                                 f'xyz="{side} {(front_y+rear_y)/2:.6f} {center_z:.6f}" '
                                 'material="body_grey" collision="${collision}"/>')
            lines.append('    <xacro:box_shape size="0.45 0.28 0.25" '
                         'xyz="0 0.05 0.10" material="body_grey" collision="${collision}"/>')
        lines.append('  </xacro:macro>')
    lines.append('</robot>')
    output.write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-dir', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path,
                        help='Output xacro; defaults to robot/part/clearing_arm_geometry.xacro')
    args = parser.parse_args()
    output = args.output or args.package_dir / 'robot/part/clearing_arm_geometry.xacro'
    generate(args.package_dir, output)


if __name__ == '__main__':
    main()

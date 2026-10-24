#!/usr/bin/env python3
"""Render an overview of the 15 navigation clouds for experiment documentation."""

from pathlib import Path
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import json

from validate_map_suite import read_pcd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path,
                        help="Image path; default docs/maps_overview.png under the package.")
    args = parser.parse_args()
    package = args.package_dir.resolve()
    records = json.loads((package / "worlds/catalog.json").read_text())["maps"]
    subtitles = {
        "flat_1": "Original indoor rooms", "flat_2": "Offset warehouse",
        "flat_3": "Asymmetric offices", "flat_4": "Plaza with props", "flat_5": "Irregular maze",
        "terrain_1": "Original terrain and rocks", "terrain_2": "Hills and rocks",
        "terrain_3": "Valley with boulders", "terrain_4": "Undulations and stones", "terrain_5": "Terraces and spires",
        "floor_1": "Flat / flat (original)", "floor_2": "Flat / west swell",
        "floor_3": "Central ridge / flat", "floor_4": "Flat / flat (new)", "floor_5": "Terraces / diagonal",
    }
    fig = plt.figure(figsize=(18, 10), facecolor="white")
    for row, category in enumerate(("flat", "terrain", "floor")):
        for column in range(5):
            name = f"{category}_{column + 1}"
            record = records[name]
            points = read_pcd(package / record["pcd"])
            points = points[::max(1, len(points) // 30000)]
            ax = fig.add_subplot(3, 5, row * 5 + column + 1, projection="3d")
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2],
                       cmap="viridis", vmin=-.5, vmax=5, s=.2, alpha=.85, linewidths=0,
                       rasterized=True)
            ax.set_title(f"{name}\n{subtitles[name]}", fontsize=11, pad=3)
            ax.view_init(elev=43, azim=-63)
            span = np.asarray(record["bounds_max"]) - record["bounds_min"]
            ax.set_box_aspect((span[0], span[1], max(6, span[2] * 2)))
            ax.set_xlabel("x (m)", fontsize=7, labelpad=-6)
            ax.set_ylabel("y (m)", fontsize=7, labelpad=-6)
            ax.set_zlabel("z (m)", fontsize=7, labelpad=-5)
            ax.tick_params(labelsize=6, pad=-2)
            ax.xaxis.set_major_locator(plt.MaxNLocator(3))
            ax.yaxis.set_major_locator(plt.MaxNLocator(3))
            ax.zaxis.set_major_locator(plt.MaxNLocator(3))
            for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
                axis.pane.fill = False
            ax.grid(alpha=.2)
    fig.suptitle("MultiMapNav: 15 simulation maps in shared world coordinates", fontsize=17, y=.975)
    fig.text(.5, .014, "Rows: flat, terrain, floor. Color encodes height. Floor subtitles list lower / upper surface types.",
             ha="center", fontsize=10)
    fig.subplots_adjust(left=.01, right=.99, top=.9, bottom=.055, wspace=.04, hspace=.23)
    destination = args.output if args.output is not None else package / "docs/maps_overview.png"
    destination = destination if destination.is_absolute() else package / destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=180)
    plt.close(fig)
    print(destination)


if __name__ == "__main__":
    main()

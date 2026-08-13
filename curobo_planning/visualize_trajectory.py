#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch


def load_obstacles(map_path):
    """Load the occupied OctoMap cells stored beside the ESDF."""
    prefix = map_path.with_name(map_path.stem.removesuffix("_esdf") + "_grid")
    with prefix.with_suffix(".json").open(encoding="utf-8") as file:
        metadata = json.load(file)
    occupancy = np.fromfile(prefix.with_suffix(".bin"), dtype=np.uint8)
    occupancy = occupancy.reshape(metadata["shape"])
    return occupancy == metadata["values"]["occupied"], metadata


def main():
    parser = argparse.ArgumentParser(description="Plot one collected trajectory.")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--map", type=Path, help="ESDF map (defaults to the dataset source map)")
    parser.add_argument("--save", type=Path)
    args = parser.parse_args()

    with h5py.File(args.dataset) as dataset:
        if not 0 <= args.index < len(dataset["goals"]):
            raise IndexError(f"Dataset contains {len(dataset['goals'])} trajectories")
        begin, end = dataset["trajectory_offsets"][args.index : args.index + 2]
        path = dataset["trajectory_points"][begin:end]
        goal = dataset["goals"][args.index]
        map_path = args.map or Path(dataset.attrs["source_map"])

    obstacles, map_metadata = load_obstacles(map_path)
    bounds_min = map_metadata["bounds_min"]
    bounds_max = map_metadata["bounds_max"]

    figure, plots = plt.subplots(1, 3, figsize=(14, 4))
    for axes, first, second, hidden, title, labels in (
        (plots[0], 0, 1, 2, "XY projection", ("world x (m)", "world y (m)")),
        (plots[1], 0, 2, 1, "XZ projection", ("world x (m)", "world z (m)")),
        (plots[2], 1, 2, 0, "YZ projection", ("world y (m)", "world z (m)")),
    ):
        axes.imshow(
            np.any(obstacles, axis=hidden).T,
            origin="lower",
            extent=[bounds_min[first], bounds_max[first], bounds_min[second], bounds_max[second]],
            cmap="Greys",
            alpha=0.35,
            interpolation="nearest",
        )
        axes.plot(path[:, first], path[:, second], label="trajectory")
        axes.scatter(path[0, first], path[0, second], label="start")
        axes.scatter(goal[first], goal[second], label="goal")
        axes.set(title=title, xlabel=labels[0], ylabel=labels[1])
        axes.axis("equal")
    handles, labels = plots[0].get_legend_handles_labels()
    plots[0].legend([Patch(color="gray", alpha=0.35), *handles], ["occupied", *labels])
    figure.tight_layout()

    if args.save:
        figure.savefig(args.save, dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()

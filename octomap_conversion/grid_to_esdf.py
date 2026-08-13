#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt


def main():
    parser = argparse.ArgumentParser(
        description="Convert a tri-state occupancy grid into a cuRobo ESDF grid."
    )
    parser.add_argument("grid", type=Path, help="Raw uint8 grid produced by octomap_to_grid")
    parser.add_argument("metadata", type=Path, help="JSON metadata produced by octomap_to_grid")
    parser.add_argument("output", type=Path, help="Output .npz file")
    args = parser.parse_args()

    with args.metadata.open(encoding="utf-8") as metadata_file:
        metadata = json.load(metadata_file)

    shape = tuple(metadata["shape"])
    occupancy = np.fromfile(args.grid, dtype=np.uint8)
    if occupancy.size != np.prod(shape):
        raise ValueError(f"Grid contains {occupancy.size} cells; metadata describes {np.prod(shape)}")
    occupancy = occupancy.reshape(shape)

    free = occupancy == metadata["values"]["free"]
    blocked = ~free  # Occupied and unobserved cells are both unsafe for planning.
    if not free.any() or not blocked.any():
        raise ValueError("The grid must contain both known-free and blocked cells")

    voxel_size = float(metadata["voxel_size"])
    free_distance = distance_transform_edt(free, sampling=voxel_size)
    blocked_distance = distance_transform_edt(blocked, sampling=voxel_size)
    half_voxel = voxel_size / 2.0
    esdf = np.where(
        free,
        free_distance - half_voxel,
        -(blocked_distance - half_voxel),
    ).astype(np.float32)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        esdf=esdf,
        center=np.asarray(metadata["center"], dtype=np.float32),
        dimensions=np.asarray(metadata["dimensions"], dtype=np.float32),
        bounds_min=np.asarray(metadata["bounds_min"], dtype=np.float32),
        bounds_max=np.asarray(metadata["bounds_max"], dtype=np.float32),
        voxel_size=np.float32(voxel_size),
        frame_id=np.asarray(metadata["frame_id"]),
    )

    print(
        f"Wrote {args.output} with shape {esdf.shape} "
        f"and ESDF range [{esdf.min():.3f}, {esdf.max():.3f}] m"
    )


if __name__ == "__main__":
    main()


# OctoMap conversion

This standalone utility converts a MoveIt OctoMap into a dense ESDF voxel grid that can be loaded by cuRobo.
It is not a ROS package and does not link against cuRobo.

## Dependencies

- CMake and a C++17 compiler
- OctoMap C++ development library
- Python 3, NumPy, and SciPy

These dependencies are already present in the project's Docker image.
For use outside the image, install the Python dependencies with:

```bash
python3 -m pip install -r octomap_conversion/requirements.txt
```

## Convert the warehouse map

From the repository root, run:

```bash
./octomap_conversion/convert.sh
```

The optional first argument selects another `.bt` file, and the optional second argument selects the output directory:

```bash
./octomap_conversion/convert.sh path/to/map.bt path/to/output
```

The wrapper uses the warehouse planning bounds `x=[-15, 15]`, `y=[-25, 25]`, and `z=[0, 12.6]` metres. Change the six bounds in `convert.sh` when converting a different world.

## Outputs

The generated `output/` directory contains:

- `warehouse_grid.bin`: dense `uint8` occupancy in C order, shaped `(x, y, z)`
- `warehouse_grid.json`: bounds, shape, resolution, frame, and voxel-value metadata
- `warehouse_esdf.npz`: signed distances and spatial metadata for the cuRobo planning workspace

The occupancy values are `0=free`, `1=occupied`, and `2=unknown`. During ESDF generation, occupied and unknown cells are both treated as blocked. The resulting ESDF is positive in known-free space and negative in blocked space.

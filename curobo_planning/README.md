# cuRobo trajectory collection

This standalone workspace uses the warehouse ESDF and cuRobo's GPU batch planner to generate collision-free X500 trajectories. It does not use ROS, MoveIt, Gazebo, or PX4.

The X500 is represented by three prismatic joints (`x`, `y`, and `z`) and six collision spheres within a 0.5 m envelope. Because there are no rotational joints, roll, pitch, and yaw remain fixed.

## Collect trajectories

First generate `octomap_conversion/output/warehouse_esdf.npz`. Then, inside the Docker container, run:

```bash
cd /workspace
python3 curobo_planning/collect_trajectories.py \
  --num-trajectories 1000 \
  --batch-size 64
```

The default output is `curobo_planning/datasets/warehouse_trajectories.h5`. Existing files are protected; use `--overwrite` when replacement is intentional.

The HDF5 file stores variable-length trajectories without resampling:

```text
goals                 (number of trajectories, 3)
trajectory_points     (all trajectory points, 3)
trajectory_offsets    (number of trajectories + 1)
```

Trajectory `i` is read with:

```python
begin, end = trajectory_offsets[i:i + 2]
trajectory = trajectory_points[begin:end]
goal = goals[i]
```

All positions use the `world` frame and are `(x, y, z)` in metres. The default start is `(0, 0, 3)`, and sampled goal heights are between `1.5` and `4.5` metres. Every orientation is the identity quaternion `(w, x, y, z) = (1, 0, 0, 0)`.

## Visualize one trajectory

```bash
python3 curobo_planning/visualize_trajectory.py \
  curobo_planning/datasets/warehouse_trajectories.h5 \
  --index 0
```

To write an image instead of opening a window, add `--save trajectory.png`.

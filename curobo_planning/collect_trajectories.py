#!/usr/bin/env python3

import argparse
from pathlib import Path

import h5py
import numpy as np
import torch

from curobo.batch_motion_planner import BatchMotionPlanner, MotionPlannerCfg
from curobo.config_io import load_yaml
from curobo.scene import Scene, VoxelGrid
from curobo.types import DeviceCfg, JointState


WORKSPACE = Path(__file__).resolve().parent


def sample_goals(
    rng, esdf, bounds_min, bounds_max, voxel_size, count, clearance, goal_z_bounds
):
    """Sample points whose ESDF clearance is large enough for the X500 sphere."""
    goals = []
    low = bounds_min + clearance
    high = bounds_max - clearance
    low[2] = max(low[2], goal_z_bounds[0])
    high[2] = min(high[2], goal_z_bounds[1])
    while len(goals) < count:
        candidates = rng.uniform(low, high, size=(count * 4, 3))
        cells = np.floor((candidates - bounds_min) / voxel_size).astype(int)
        safe = esdf[cells[:, 0], cells[:, 1], cells[:, 2]] > clearance
        goals.extend(candidates[safe])
    return np.asarray(goals[:count], dtype=np.float32)


def append_trajectories(file, goals, trajectories):
    """Append ragged trajectories using one point array and an offsets array."""
    old_goals = len(file["goals"])
    old_points = len(file["trajectory_points"])
    points = np.concatenate(trajectories)

    file["goals"].resize(old_goals + len(goals), axis=0)
    file["goals"][old_goals:] = goals
    file["trajectory_points"].resize(old_points + len(points), axis=0)
    file["trajectory_points"][old_points:] = points
    file["trajectory_offsets"].resize(old_goals + len(goals) + 1, axis=0)
    file["trajectory_offsets"][old_goals + 1:] = old_points + np.cumsum(
        [len(path) for path in trajectories]
    )


def main():
    parser = argparse.ArgumentParser(description="Generate X500 trajectories with cuRobo.")
    parser.add_argument(
        "--map",
        type=Path,
        default=WORKSPACE.parent / "octomap_conversion/output/warehouse_esdf.npz",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=WORKSPACE / "datasets/warehouse_trajectories.h5",
    )
    parser.add_argument("--num-trajectories", type=int, default=1000)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--start", type=float, nargs=3, default=[0.0, 0.0, 3.0])
    parser.add_argument("--goal-z-range", type=float, default=1.5)
    parser.add_argument("--clearance", type=float, default=0.55)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable. Start the container with --gpus all.")
    if args.output.exists() and not args.overwrite:
        raise FileExistsError(f"{args.output} exists; pass --overwrite to replace it")

    map_data = np.load(args.map)
    esdf = map_data["esdf"]
    bounds_min = map_data["bounds_min"]
    bounds_max = map_data["bounds_max"]
    voxel_size = float(map_data["voxel_size"])
    goal_z_bounds = [args.start[2] - args.goal_z_range, args.start[2] + args.goal_z_range]

    device = DeviceCfg(device=torch.device("cuda:0"), dtype=torch.float32)
    voxel = VoxelGrid(
        name="warehouse",
        pose=[*map_data["center"].tolist(), 1.0, 0.0, 0.0, 0.0],
        dims=map_data["dimensions"].tolist(),
        voxel_size=voxel_size,
        feature_tensor=torch.as_tensor(esdf, device="cuda", dtype=torch.float16),
        feature_dtype=torch.float16,
    )

    robot = load_yaml(str(WORKSPACE / "config/x500_3dof.yml"))
    robot["robot_cfg"]["kinematics"]["urdf_path"] = str(
        WORKSPACE / "config/x500_3dof.urdf"
    )
    robot["robot_cfg"]["kinematics"]["asset_root_path"] = str(WORKSPACE / "config")

    planner_config = MotionPlannerCfg.create(
        robot=robot,
        scene_model=Scene(voxel=[voxel]),
        device_cfg=device,
        max_batch_size=args.batch_size,
        multi_env=False,
        self_collision_check=False,
        random_seed=args.seed,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    with h5py.File(args.output, "w") as dataset, BatchMotionPlanner(planner_config) as planner:
        dataset.create_dataset("goals", (0, 3), maxshape=(None, 3), dtype="f4")
        dataset.create_dataset("trajectory_points", (0, 3), maxshape=(None, 3), dtype="f4")
        dataset.create_dataset("trajectory_offsets", data=np.array([0], dtype=np.int64), maxshape=(None,))
        dataset.attrs["frame_id"] = str(map_data["frame_id"])
        dataset.attrs["start"] = args.start
        dataset.attrs["goal_z_bounds"] = goal_z_bounds
        dataset.attrs["fixed_orientation_wxyz"] = [1.0, 0.0, 0.0, 0.0]
        dataset.attrs["source_map"] = str(args.map)

        planner.warmup(enable_graph=True, num_warmup_iterations=1)
        start = torch.tensor(args.start, **device.as_torch_dict()).repeat(args.batch_size, 1)
        current_states = JointState.from_position(start, joint_names=planner.joint_names)

        while len(dataset["goals"]) < args.num_trajectories:
            goals = sample_goals(
                rng, esdf, bounds_min, bounds_max, voxel_size,
                args.batch_size, args.clearance, goal_z_bounds,
            )
            goal_states = JointState.from_position(
                torch.as_tensor(goals, **device.as_torch_dict()),
                joint_names=planner.joint_names,
            )
            result = planner.plan_cspace(
                goal_states, current_states, max_attempts=3, enable_graph_attempt=0
            )
            if result is None:
                continue

            success = result.success[:, 0].detach().cpu().numpy()
            positions = result.interpolated_trajectory.position[:, 0].detach().cpu().numpy()
            lengths = result.interpolated_last_tstep[:, 0].detach().cpu().numpy()
            successful_goals = goals[success]
            trajectories = [positions[i, : int(lengths[i]), :3] for i in np.flatnonzero(success)]

            remaining = args.num_trajectories - len(dataset["goals"])
            if trajectories:
                append_trajectories(
                    dataset, successful_goals[:remaining], trajectories[:remaining]
                )
                dataset.flush()
            print(f"Collected {len(dataset['goals'])}/{args.num_trajectories}")

    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()

#!/usr/bin/env bash
set -eo pipefail

if [[ $# -gt 3 ]]; then
    echo "Usage: $0 [dataset.h5] [first_trajectory_index] [number_of_trajectories (0=all)]"
    exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(dirname "$script_dir")/drone_ws"
dataset="${1:-/workspace/curobo_planning/diffusion_outputs/diffusion_test_trajectories.h5}"
trajectory_index="${2:-0}"
num_trajectories="${3:-0}"

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

if [[ ! -f "$dataset" ]]; then
    echo "error: dataset not found: $dataset" >&2
    exit 1
fi

ros2 launch x500_moveit_config static_octomap_demo.launch.py \
    start_x:=2.0 start_y:=-6.0 start_z:=2.5 start_yaw:=0.0 &
launch_pid=$!
trap 'kill "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true' EXIT

ros2 run curobo_trajectory_visualizer visualize_trajectory --ros-args \
    -p dataset:="$dataset" \
    -p trajectory_index:="$trajectory_index" \
    -p num_trajectories:="$num_trajectories"

#!/bin/bash

# Starts MoveIt without Gazebo or live perception and loads the saved depot
# OctoMap through static_octomap_demo.launch.py.
#
# Optional launch arguments can be appended, for example:
#   ./launch_static_octomap_demo.sh start_z:=3.0 start_yaw:=0.0

set -eo pipefail

ROS_DISTRO="humble"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE_ROOT="${PROJECT_ROOT}/drone_ws"
WORKSPACE_INSTALL="${WORKSPACE_ROOT}/install/setup.bash"
MAP_FILE="${WORKSPACE_ROOT}/src/trajectory_dataset_collector/maps/depot.bt"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "$WORKSPACE_INSTALL" ]]; then
    echo "error: $WORKSPACE_INSTALL not found. Build the workspace first:" >&2
    echo "  cd $WORKSPACE_ROOT && colcon build --symlink-install" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$WORKSPACE_INSTALL"
set -u

if [[ ! -s "$MAP_FILE" ]]; then
    echo "error: saved OctoMap not found or empty: $MAP_FILE" >&2
    exit 1
fi

if ! ros2 pkg prefix x500_moveit_config >/dev/null 2>&1; then
    echo "error: x500_moveit_config is not installed in the sourced workspace" >&2
    echo "  cd $WORKSPACE_ROOT" >&2
    echo "  colcon build --packages-select x500_moveit_config --symlink-install" >&2
    exit 1
fi

echo "Starting static MoveIt demo with OctoMap: $MAP_FILE"
exec ros2 launch x500_moveit_config static_octomap_demo.launch.py \
    map_file:="$MAP_FILE" \
    "$@"

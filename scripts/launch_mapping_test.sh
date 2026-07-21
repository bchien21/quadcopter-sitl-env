#!/bin/bash

# Launches MoveIt (demo.launch.py), waits for it to settle, then starts the
# offboard controller so the moving depth camera can populate the OctoMap.
#
# Usage: run this from a *new* tmux pane inside px4_session, after
# startup.sh's PX4/Gazebo pane has stabilized (vehicle spawned, bridge
# topics flowing). It does not manage tmux itself, since it's meant to be
# started manually once you decide the sim is ready:
#
#   tmux split-window -t px4_session   # or Ctrl-b " / Ctrl-b %
#   bash /workspace/scripts/launch_mapping_test.sh
#
# Sourcing is done explicitly here (rather than relying on ~/.bashrc)
# because this runs as a non-interactive script.

set -e

ROS_DISTRO="humble"
WORKSPACE_INSTALL="/workspace/drone_ws/install/setup.bash"
MOVEIT_SETTLE_SECONDS="${MOVEIT_SETTLE_SECONDS:-10}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "$WORKSPACE_INSTALL" ]]; then
    echo "error: $WORKSPACE_INSTALL not found. Build the workspace first:" >&2
    echo "  cd /workspace/drone_ws && colcon build --symlink-install" >&2
    exit 1
fi
source "$WORKSPACE_INSTALL"

moveit_pid=""
offboard_pid=""

cleanup() {
    trap - EXIT INT TERM
    [[ -z "$offboard_pid" ]] || kill "$offboard_pid" 2>/dev/null || true
    [[ -z "$moveit_pid" ]] || kill "$moveit_pid" 2>/dev/null || true
    [[ -z "$offboard_pid" ]] || wait "$offboard_pid" 2>/dev/null || true
    [[ -z "$moveit_pid" ]] || wait "$moveit_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch x500_moveit_config demo.launch.py &
moveit_pid=$!

echo "Waiting ${MOVEIT_SETTLE_SECONDS}s for MoveIt and the OctoMap updater to settle..."
sleep "$MOVEIT_SETTLE_SECONDS"

if ! kill -0 "$moveit_pid" 2>/dev/null; then
    echo "error: MoveIt exited before the mapping test could start" >&2
    wait "$moveit_pid" || true
    exit 1
fi

ros2 run px4_offboard_cpp offboard_control &
offboard_pid=$!

# Keep both processes attached to this script. If either exits, stop the other.
wait -n "$moveit_pid" "$offboard_pid"

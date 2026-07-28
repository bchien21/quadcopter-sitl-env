#!/bin/bash

# Launches MoveIt (demo.launch.py), waits for move_group to become ready and
# the OctoMap updater to settle, then starts the deterministic PX4 mapping
# mission so the moving depth camera can populate the OctoMap.
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
MOVEIT_READY_TIMEOUT_SECONDS="${MOVEIT_READY_TIMEOUT_SECONDS:-60}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "$WORKSPACE_INSTALL" ]]; then
    echo "error: $WORKSPACE_INSTALL not found. Build the workspace first:" >&2
    echo "  cd /workspace/drone_ws && colcon build --symlink-install" >&2
    exit 1
fi
source "$WORKSPACE_INSTALL"

moveit_pid=""
mission_pid=""

cleanup() {
    trap - EXIT INT TERM
    [[ -z "$mission_pid" ]] || kill "$mission_pid" 2>/dev/null || true
    [[ -z "$moveit_pid" ]] || kill "$moveit_pid" 2>/dev/null || true
    [[ -z "$mission_pid" ]] || wait "$mission_pid" 2>/dev/null || true
    [[ -z "$moveit_pid" ]] || wait "$moveit_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if ! ros2 pkg executables px4_offboard_cpp |
    grep -Eq '^px4_offboard_cpp[[:space:]]+octomap_mapping_mission$'; then
    echo "error: octomap_mapping_mission is not installed. Build it first:" >&2
    echo "  cd /workspace/drone_ws" >&2
    echo "  colcon build --packages-select px4_offboard_cpp --symlink-install" >&2
    exit 1
fi

ros2 launch x500_moveit_config demo.launch.py &
moveit_pid=$!

echo "Waiting for MoveIt's /get_planning_scene service..."
ready_deadline=$((SECONDS + MOVEIT_READY_TIMEOUT_SECONDS))
while ! ros2 service list 2>/dev/null | grep -q '^/get_planning_scene$'; do
    if ! kill -0 "$moveit_pid" 2>/dev/null; then
        echo "error: MoveIt exited before becoming ready" >&2
        wait "$moveit_pid" || true
        exit 1
    fi

    if (( SECONDS >= ready_deadline )); then
        echo "error: MoveIt did not become ready within ${MOVEIT_READY_TIMEOUT_SECONDS}s" >&2
        exit 1
    fi

    sleep 1
done

echo "MoveIt is ready. Waiting ${MOVEIT_SETTLE_SECONDS}s for the OctoMap updater to settle..."
echo "Arm the vehicle in QGroundControl before the mapping mission starts."
sleep "$MOVEIT_SETTLE_SECONDS"

if ! kill -0 "$moveit_pid" 2>/dev/null; then
    echo "error: MoveIt exited before the mapping test could start" >&2
    wait "$moveit_pid" || true
    exit 1
fi

echo "Starting deterministic PX4 mapping mission..."
ros2 run px4_offboard_cpp octomap_mapping_mission &
mission_pid=$!

# Keep both processes attached to this script. If either exits, stop the other.
wait -n "$moveit_pid" "$mission_pid"

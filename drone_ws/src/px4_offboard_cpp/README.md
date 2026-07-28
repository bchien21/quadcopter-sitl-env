# PX4 offboard nodes

## Deterministic OctoMap mapping mission

`octomap_mapping_mission` is a minimal timed Offboard controller adapted from
`offboard_control.cpp`. It continuously publishes position setpoints and
requests Offboard mode after one second. It does not arm the vehicle and does
not consume PX4 feedback; arm the vehicle manually in QGroundControl before
starting the node.

The four hardcoded PX4 local-NED waypoints form a 4 m square at 2.5 m altitude:

```text
NED (0, 0, -2.5)
NED (4, 0, -2.5)
NED (4, 4, -2.5)
NED (0, 4, -2.5)
```

The node changes targets at 5, 10, and 15 seconds after startup. After the
fourth target is commanded at 15 seconds, the node holds it until it receives
a manual setpoint. Verify that the corresponding square is collision-free in
Gazebo before running it.

Build and run:

```bash
cd /workspace/drone_ws
colcon build --packages-select px4_offboard_cpp --symlink-install
source install/setup.bash
ros2 run px4_offboard_cpp octomap_mapping_mission
```

After the four-waypoint sequence, send additional PX4 local-NED position and
yaw targets from another terminal:

```bash
ros2 topic pub --once \
  /octomap_mapping_mission/manual_setpoint \
  px4_msgs/msg/TrajectorySetpoint \
  "{position: [6.0, 2.0, -3.0], yaw: 1.57}"
```

The position is `[north, east, down]` in metres, so a negative third component
commands an altitude above the local origin. Yaw is an absolute NED heading in
radians: `0` is north, `1.57` is east, `3.14` is south, and `-1.57` is west.
Each valid command replaces the previous manual target and is continuously
published at 10 Hz. Commands received before the timed sequence completes, or
commands containing non-finite position or yaw values, are ignored.

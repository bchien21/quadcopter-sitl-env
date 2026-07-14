#!/bin/bash

# Define the tmux session name
SESSION_NAME="px4_session"

PX4_DIR="/workspace/PX4-Autopilot"
ROS_DISTRO="humble"

# 1. Start Session
tmux new-session -d -s $SESSION_NAME

# 2. Pane 0 (Left): PX4 Simulation
tmux send-keys -t $SESSION_NAME:0.0 "cd $PX4_DIR" C-m
tmux send-keys -t $SESSION_NAME:0.0 "make px4_sitl gz_x500_depth" C-m
#tmux send-keys -t $SESSION_NAME:0.0 "HEADLESS=1 make px4_sitl gz_x500_depth" C-m

# 3. Pane 1 (Top Right): MicroXRCE Agent
tmux split-window -h -t $SESSION_NAME:0.0
tmux resize-pane -t $SESSION_NAME:0.0 -x 60%
tmux send-keys -t $SESSION_NAME:0.1 "MicroXRCEAgent udp4 -p 8888" C-m

# 4. Pane 2 (Bottom Right): ROS Bridge
tmux split-window -v -t $SESSION_NAME:0.1

# Define Bridge Arguments (Gazebo -> ROS)
# Using line continuation (\) for readability
# /camera_info is the depth camera's info topic (gz publishes it at root scope,
# next to the sensor's <topic>depth_camera</topic>).
BRIDGE_ARGS="/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image \
/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"

# BRIDGE_ARGS="/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
# /depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
# /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
# /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
# /world/default/control@ros_gz_interfaces/srv/ControlWorld \
# /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose"

# Remaps: depth image + its camera_info as sibling topics (image_transport
# expects camera_info in the same namespace as the image topic; MoveIt's
# DepthImageOctomapUpdater relies on this to find the intrinsics).
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run ros_gz_bridge parameter_bridge $BRIDGE_ARGS --ros-args \
    -r /depth_camera:=/depth_camera/depth/image_raw \
    -r /camera_info:=/depth_camera/depth/camera_info" C-m

# 5. Pane 3 (Bottom Left): Static TF Publisher (world -> camera_link)
# tmux split-window -v -t $SESSION_NAME:0.2
# # +90 deg about +Y (right-hand) between world and camera_link
# tmux send-keys -t $SESSION_NAME:0.3 "ros2 run tf2_ros static_transform_publisher --x 0.12 --y 0.03 --z 0.0 --qx 0 --qy 0.7071067812 --qz 0 --qw 0.7071067812 --frame-id world --child-frame-id camera_link" C-m

# Attach
tmux set -g mouse on
tmux attach-session -t $SESSION_NAME
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    start_z = LaunchConfiguration("start_z")
    start_yaw = LaunchConfiguration("start_yaw")

    # Deliberately omit sensors_3d.yaml. This gives MoveIt an occupancy map
    # monitor with save/load services, but no updater that can modify the
    # loaded map from live point-cloud data.
    moveit_config = MoveItConfigsBuilder(
        "x500_with_depth_camera",
        package_name="x500_moveit_config",
    ).to_moveit_configs()
    demo_launch = generate_demo_launch(moveit_config)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("moveit_octomaps"), "maps", "warehouse.bt"]
                ),
                description="Absolute path to the saved binary OctoMap (.bt)",
            ),
            DeclareLaunchArgument(
                "start_x",
                default_value="0.0",
                description="Fixed dataset start X position in the world frame (m)",
            ),
            DeclareLaunchArgument(
                "start_y",
                default_value="0.0",
                description="Fixed dataset start Y position in the world frame (m)",
            ),
            DeclareLaunchArgument(
                "start_z",
                default_value="2.5",
                description="Fixed dataset start Z position in the world frame (m)",
            ),
            DeclareLaunchArgument(
                "start_yaw",
                default_value="1.5708",
                description="Fixed dataset start ENU yaw in radians",
            ),
            # This launch is independent of Gazebo and therefore uses the
            # system clock rather than waiting for the simulated /clock topic.
            SetParameter(name="use_sim_time", value=False),
            *demo_launch.entities,
            # The static transform represents only the current/fixed dataset
            # start state. MoveIt computes transforms for candidate trajectory
            # states internally through the floating virtual joint.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_dataset_start_tf",
                output="screen",
                arguments=[
                    "--x",
                    start_x,
                    "--y",
                    start_y,
                    "--z",
                    start_z,
                    "--yaw",
                    start_yaw,
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            # ros2 service call waits for /load_map to become available, so
            # loading does not depend on a fixed startup delay.
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/load_map",
                    "moveit_msgs/srv/LoadMap",
                    ["{filename: '", map_file, "'}"],
                ],
                name="load_static_octomap",
                output="screen",
            ),
        ]
    )

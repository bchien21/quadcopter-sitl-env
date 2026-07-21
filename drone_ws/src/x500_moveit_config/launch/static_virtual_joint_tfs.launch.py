from launch import LaunchDescription


def generate_launch_description():
    # world -> base_link is published dynamically from the Gazebo vehicle pose.
    # Keep this file because MoveIt's generated demo includes it by name, but do
    # not start the generated identity static_transform_publisher.
    return LaunchDescription()

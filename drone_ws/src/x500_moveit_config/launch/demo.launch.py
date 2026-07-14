import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("x500_with_depth_camera", package_name="x500_moveit_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("x500_with_depth_camera", package_name="x500_moveit_config").sensors_3d(
            file_path=os.path.join(get_package_share_directory("x500_moveit_config"),"config/sensors_3d.yaml",)).to_moveit_configs()
    )
    ld = generate_demo_launch(moveit_config)

    # Back-project the bridged depth image into a point cloud for the
    # PointCloudOctomapUpdater. The resulting cloud is stamped with the image's
    # optical frame and its points actually use the optical convention, unlike
    # the point cloud published by Gazebo (X-forward data in an optical frame).
    ld.add_action(
        Node(
            package="depth_image_proc",
            executable="point_cloud_xyz_node",
            name="depth_to_pointcloud",
            output="screen",
            remappings=[
                ("image_rect", "/depth_camera/depth/image_raw"),
                ("camera_info", "/depth_camera/depth/camera_info"),
                ("points", "/depth_camera/points"),
            ],
        )
    )
    return ld

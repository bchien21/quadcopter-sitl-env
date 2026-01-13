import os
from ament_index_python.packages import get_package_share_directory
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
    return generate_demo_launch(moveit_config)

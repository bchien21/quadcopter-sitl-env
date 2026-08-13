cmake_minimum_required(VERSION 3.8)
project(curobo_trajectory_visualizer)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/visualize_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME visualize_trajectory
)

ament_package()

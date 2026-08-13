cmake_minimum_required(VERSION 3.8)
project(px4_offboard_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(offboard_control src/offboard_control.cpp)
target_include_directories(offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(offboard_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  offboard_control
  "rclcpp"
  "px4_msgs"
  "std_msgs"
)

add_executable(octomap_mapping_mission src/octomap_mapping_mission.cpp)
target_compile_features(octomap_mapping_mission PUBLIC cxx_std_17)
ament_target_dependencies(
  octomap_mapping_mission
  ament_index_cpp
  rclcpp
  px4_msgs
)

add_executable(gazebo_pose_tf src/gazebo_pose_tf.cpp)
target_compile_features(gazebo_pose_tf PUBLIC cxx_std_17)
ament_target_dependencies(
  gazebo_pose_tf
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)
target_link_libraries(
  gazebo_pose_tf
  gz-msgs10::core
  gz-transport13::core
)

install(TARGETS offboard_control octomap_mapping_mission gazebo_pose_tf
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

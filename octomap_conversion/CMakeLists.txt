cmake_minimum_required(VERSION 3.16)
project(octomap_conversion LANGUAGES CXX)

find_package(octomap REQUIRED)

add_executable(octomap_to_grid octomap_to_grid.cpp)
target_compile_features(octomap_to_grid PRIVATE cxx_std_17)
target_include_directories(octomap_to_grid PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_to_grid PRIVATE ${OCTOMAP_LIBRARIES})


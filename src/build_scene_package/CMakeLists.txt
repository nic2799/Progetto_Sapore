cmake_minimum_required(VERSION 3.8)
project(build_scene_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometric_shapes REQUIRED)
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
find_package(shape_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)



# Add executable per box semplice
add_executable(build_scene_simple src/build_scene_simple.cpp)

# Dipendenze per build_scene_simple
ament_target_dependencies(build_scene_simple
  rclcpp
  moveit_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
  moveit_ros_planning_interface
  yaml-cpp
  tf2
  ament_index_cpp
)



# Install build_scene_simple
install(TARGETS
  build_scene_simple
  DESTINATION lib/${PROJECT_NAME}
)




target_link_libraries(build_scene_simple ${YAML_CPP_LIBRARIES})



# Add executable per caricamento mesh
add_executable(build_scene_mesh src/build_scene_mesh.cpp)

# Dipendenze per build_scene_mesh
ament_target_dependencies(build_scene_mesh
  rclcpp
  moveit_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
  moveit_ros_planning_interface
  yaml-cpp
  tf2
  ament_index_cpp
)

# Install build_scene_mesh
install(TARGETS
  build_scene_mesh
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(build_scene_mesh ${YAML_CPP_LIBRARIES})

# Install configuration files
install(DIRECTORY
  obstacle_descritpion
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
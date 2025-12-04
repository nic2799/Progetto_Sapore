cmake_minimum_required(VERSION 3.8)
project(validate_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(control_msgs REQUIRED)





# Executable ExecuteYamlTrajectoriesNode
add_executable(execute_yaml_trajectories_node src/ExecuteYamlTrajectoriesNode.cpp)

ament_target_dependencies(execute_yaml_trajectories_node
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
  control_msgs
)

target_link_libraries(execute_yaml_trajectories_node yaml-cpp)


# Executable Valid_trajectory_click
add_executable(Valid_trajectory_click src/Valid_trajectory_click.cpp)



ament_target_dependencies(Valid_trajectory_click
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
)

target_link_libraries(Valid_trajectory_click yaml-cpp)

# Install
install(TARGETS
  Valid_trajectory_click
  execute_yaml_trajectories_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


cmake_minimum_required(VERSION 3.8)
project(nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)#per obs forse inutili anche quelli di seguito
find_package(visualization_msgs REQUIRED)#per obs
find_package(tf2_ros REQUIRED)#per obs
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)#per obs
find_package(tf2_eigen REQUIRED)#per obs
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)#per obs
find_package(geometry_msgs REQUIRED)#per obs forse inutili anche quelli di seguito
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_executable(task src/task.cpp)
ament_target_dependencies(task 
  rclcpp 
  rclcpp_action 
  sensor_msgs 
  interfaces
)
# Original Jtraj node
add_executable(jtraj_node src/Jtraj.cpp)
ament_target_dependencies(jtraj_node 
  rclcpp 
  rclcpp_action 
  sensor_msgs 
  interfaces
)

# Left arm Jtraj node
add_executable(jtraj_left_node src/JtrajLeft.cpp)
ament_target_dependencies(jtraj_left_node 
  rclcpp 
  rclcpp_action 
  sensor_msgs 
  interfaces
)

# Right arm Jtraj node
add_executable(jtraj_right_node src/JtrajRight.cpp)
ament_target_dependencies(jtraj_right_node 
  rclcpp 
  rclcpp_action 
  sensor_msgs 
  interfaces
)

add_executable(Reach src/test_reach.cpp)
ament_target_dependencies(Reach 
  rclcpp 
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
  )


install(TARGETS
  jtraj_node
  jtraj_left_node
  jtraj_right_node
  task
  
  Reach
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


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

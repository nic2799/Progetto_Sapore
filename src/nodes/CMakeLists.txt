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


install(TARGETS
  jtraj_node
  jtraj_left_node
  jtraj_right_node
  task
  
  DESTINATION lib/${PROJECT_NAME}
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

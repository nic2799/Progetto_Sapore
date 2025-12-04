cmake_minimum_required(VERSION 3.8)
project(robot_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(yaml-cpp REQUIRED)


add_library(SaveYaml src/SaveYaml.cpp)
target_include_directories(SaveYaml PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(SaveYaml
  moveit_ros_planning_interface
  geometry_msgs
  yaml-cpp
  rclcpp
)

target_link_libraries(SaveYaml
  yaml-cpp    # <--- QUESTA È LA PARTE CHE MANCAVA
)

# MoveitHelper library
add_library(MoveitHelper src/MoveitHelper.cpp)
target_include_directories(MoveitHelper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(MoveitHelper
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  rclcpp
  tf2_geometry_msgs
)

# Eseguibile pick_place_server
add_executable(pick_place_server src/pick_place_server.cpp)
ament_target_dependencies(pick_place_server
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_srvs
  moveit_msgs
  visualization_msgs
  geometric_shapes
)
target_link_libraries(pick_place_server SaveYaml MoveitHelper)
target_include_directories(MoveitHelper PUBLIC include)
target_include_directories(SaveYaml PUBLIC include)

# Installazione
install(TARGETS
  pick_place_server
  SaveYaml
  MoveitHelper
  DESTINATION lib/${PROJECT_NAME}
)

# Installa launch files
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

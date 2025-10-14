# Install script for directory: /home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/src/nodes

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/install/nodes")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nodes" TYPE EXECUTABLE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/jtraj_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_node"
         OLD_RPATH "/opt/ros/jazzy/lib:/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/install/interfaces/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/CMakeFiles/jtraj_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_left_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_left_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_left_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nodes" TYPE EXECUTABLE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/jtraj_left_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_left_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_left_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_left_node"
         OLD_RPATH "/opt/ros/jazzy/lib:/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/install/interfaces/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_left_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/CMakeFiles/jtraj_left_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_right_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_right_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_right_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nodes" TYPE EXECUTABLE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/jtraj_right_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_right_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_right_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_right_node"
         OLD_RPATH "/opt/ros/jazzy/lib:/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/install/interfaces/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/jtraj_right_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/CMakeFiles/jtraj_right_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/task" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/task")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/task"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nodes" TYPE EXECUTABLE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/task")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/task" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/task")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/task"
         OLD_RPATH "/opt/ros/jazzy/lib:/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/install/interfaces/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/nodes/task")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/CMakeFiles/task.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/nodes")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/nodes")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes/environment" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes/environment" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_index/share/ament_index/resource_index/packages/nodes")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes/cmake" TYPE FILE FILES
    "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_core/nodesConfig.cmake"
    "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/ament_cmake_core/nodesConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nodes" TYPE FILE FILES "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/src/nodes/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/build/nodes/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

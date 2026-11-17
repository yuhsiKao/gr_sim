# Written for building the robot's interface definitions (copied from ~/current_ros2/share/ghost_manager_interfaces
# on the robot, whose install space has no CMakeLists.txt) on a development machine.
cmake_minimum_required(VERSION 3.8)
project(ghost_manager_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB interface_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS msg/*.msg srv/*.srv action/*.action)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

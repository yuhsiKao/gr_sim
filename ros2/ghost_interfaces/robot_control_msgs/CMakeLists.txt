# Written for building the robot's interface definitions (copied from ~/current_ros2/share/robot_control_msgs
# on the robot, whose install space has no CMakeLists.txt) on a development machine.
cmake_minimum_required(VERSION 3.8)
project(robot_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

file(GLOB interface_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS msg/*.msg srv/*.srv action/*.action)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs action_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

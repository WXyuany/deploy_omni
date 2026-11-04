cmake_minimum_required(VERSION 3.8)
project(interface_protocol)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  msg/JointCommand.msg
  msg/JointState.msg
  msg/ImuInfo.msg
  msg/MotionState.msg
  msg/PowerInfo.msg
  msg/Alert.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

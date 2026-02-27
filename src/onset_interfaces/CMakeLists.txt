cmake_minimum_required(VERSION 3.8)
project(onset_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaunchCommand.msg"
  "msg/OnsetStatus.msg"
  "msg/STM32Message.msg"
  "msg/STM32State.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
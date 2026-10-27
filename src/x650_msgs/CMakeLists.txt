cmake_minimum_required(VERSION 3.16)
project(x650_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrameMetadata.msg"
  "msg/KeyframeEvent.msg"
  "msg/KeyframePose.msg"
  "msg/PoseUpdateBatch.msg"
  "msg/MappingHealth.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(coal_platform_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SimHeader.msg"
  "msg/RunContext.msg"
  "msg/GroundTruth.msg"
  "msg/RobotState.msg"
  "msg/UwbRangeRaw.msg"
  "msg/LidarFrameRaw.msg"
  "msg/Intersection.msg"
  "msg/ReflectorCandidate.msg"
  "msg/UwbDecision.msg"
  "msg/UwbRangeObserved.msg"
  "msg/LidarDecision.msg"
  "msg/LidarBeamState.msg"
  "msg/LidarFrameObserved.msg"
  "srv/QueryRay.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

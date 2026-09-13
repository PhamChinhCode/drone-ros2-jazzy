cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionWaypoint.msg"
  "msg/MissionPlan.msg"
  "msg/MissionState.msg"
  "msg/FcCommand.msg"
  "msg/GripperCommand.msg"
  "msg/GripperStatus.msg"
  "msg/MarkerQuality.msg"
  "msg/EkfHealth.msg"
  "msg/FailsafeEvent.msg"
  "msg/TelemetryPacket.msg"
  "srv/Arm.srv"
  "srv/Takeoff.srv"
  "srv/GotoWaypoint.srv"
  "srv/FcSimpleCommand.srv"
  "srv/EmergencyDisarm.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()

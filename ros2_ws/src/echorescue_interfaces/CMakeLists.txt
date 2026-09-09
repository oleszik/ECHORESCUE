cmake_minimum_required(VERSION 3.10)
project(echorescue_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CellObservation.msg"
  "msg/SurvivorEvidence.msg"
  "msg/SensorObservation.msg"
  "msg/AgentState.msg"
  "msg/BridgeStatus.msg"
  "msg/MavlinkVehicleState.msg"
  "msg/MavlinkLocalPositionNed.msg"
  "msg/MavlinkGlobalPosition.msg"
  "msg/MavlinkTelemetryStatus.msg"
  "action/MoveGrid.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

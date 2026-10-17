cmake_minimum_required(VERSION 3.8)
project(fr10_motion_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Insert.action"
  "action/Move.action"
  "msg/ControllerState.msg"
  "msg/MotionCommandStatus.msg"
  "msg/JointTrajectoryCommand.msg"
  "msg/WrenchSourceStatus.msg"
  "msg/VehicleParkingLease.msg"
  "msg/VehicleParkingState.msg"
  "srv/TareWrench.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
    trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

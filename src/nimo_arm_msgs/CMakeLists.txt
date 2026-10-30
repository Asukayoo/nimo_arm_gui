cmake_minimum_required(VERSION 3.8)
project(nimo_arm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/DualArmStatus.msg"
  "msg/JogCart.msg"
  "msg/JogJoint.msg"
  "msg/ServoPByPico.msg"
  "msg/Waypoint.msg"
  "srv/SetControlMode.srv"
  "srv/SetTeleopMotion.srv"
  "srv/SetBrake.srv"
  "srv/SetJointImpKd.srv"
  "srv/SetCartImpKd.srv"
  "srv/WaypointSave.srv"
  "srv/WaypointDelete.srv"
  "srv/WaypointList.srv"
  "srv/WaypointGoto.srv"
  "srv/RecordStart.srv"
  "srv/RecordStop.srv"
  "srv/TrajRecordStart.srv"
  "srv/TrajRecordStop.srv"
  "srv/TrajPostprocess.srv"
  "srv/TrajList.srv"
  "srv/TrajReplayStart.srv"
  "srv/TrajReplayStop.srv"
  "action/MovJ.action"
  "action/MovL.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

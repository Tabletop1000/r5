cmake_minimum_required(VERSION 3.8)
project(r5_dashboard_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Alert.msg"
  "msg/AlertArray.msg"
  "msg/RobotStats.msg"
  "srv/AcknowledgeAlert.srv"
  "action/MotorTest.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

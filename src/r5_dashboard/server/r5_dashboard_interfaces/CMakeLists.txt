cmake_minimum_required(VERSION 3.8)
project(r5_dashboard_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Alert.msg"
  "msg/AlertArray.msg"
  "msg/Check.msg"
  "msg/CoveragePlan.msg"
  "msg/Field.msg"
  "msg/GeoPolygon.msg"
  "msg/JobStatus.msg"
  "msg/LatLon.msg"
  "msg/PathSection.msg"
  "msg/RecordingStatus.msg"
  "msg/RobotSettings.msg"
  "msg/RobotStats.msg"
  "msg/SpraySettings.msg"
  "srv/AcknowledgeAlert.srv"
  "srv/DeleteField.srv"
  "srv/GetHome.srv"
  "srv/GetJobDefaults.srv"
  "srv/ListFields.srv"
  "srv/PlanJob.srv"
  "srv/ResumeJob.srv"
  "srv/SaveField.srv"
  "srv/SetHome.srv"
  "srv/SetSettings.srv"
  "srv/StartJob.srv"
  "action/MotorTest.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

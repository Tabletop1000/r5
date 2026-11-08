cmake_minimum_required(VERSION 3.8)
project(r5_dashboard_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(r5_dashboard_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# Only on the robot, where the u-blox driver is installed. Without it, field_manager can't tell RTK
# float from fixed when the receiver only reports GBAS, as before.
find_package(ublox_msgs QUIET)

# The dashboard's logic, without ROS, so it can be unit tested.
add_library(dashboard_core STATIC
  src/fields.cpp
  src/files.cpp
  src/geo.cpp
  src/motor_test.cpp
  src/settings.cpp
  src/stats.cpp
)
set_target_properties(dashboard_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(dashboard_core PUBLIC cxx_std_17)
target_include_directories(dashboard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(dashboard_core PUBLIC nlohmann_json::nlohmann_json)

# Every node is a component, loaded into dashboard.launch.py's container.
add_library(${PROJECT_NAME} SHARED
  src/field_manager_node.cpp
  src/motor_test_node.cpp
  src/robot_status_node.cpp
  src/ping.cpp
  src/sprayer_stub.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  dashboard_core
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${r5_dashboard_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)
if(ublox_msgs_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE R5_HAVE_UBLOX_MSGS)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${ublox_msgs_TARGETS})
endif()
rclcpp_components_register_nodes(${PROJECT_NAME}
  "r5_dashboard::FieldManagerNode"
  "r5_dashboard::MotorTestNode"
  "r5_dashboard::Ping"
  "r5_dashboard::RobotStatusNode"
  "r5_dashboard::SprayerStub"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dashboard_core
    test/test_geo_fields.cpp
    test/test_motor_test.cpp
    test/test_settings.cpp
    test/test_stats.cpp
  )
  target_link_libraries(test_dashboard_core dashboard_core)
endif()

ament_package()

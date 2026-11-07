cmake_minimum_required(VERSION 3.8)
project(r5_dashboard_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Every node is a component, loaded into dashboard.launch.py's container.
add_library(${PROJECT_NAME} SHARED
  src/ping.cpp
  src/sprayer_stub.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)
rclcpp_components_register_nodes(${PROJECT_NAME} "r5_dashboard::Ping" "r5_dashboard::SprayerStub")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

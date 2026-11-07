cmake_minimum_required(VERSION 3.8)
project(r5_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Reading the computer's state, without ROS, so it can be tested anywhere.
add_library(system_info STATIC src/system_info.cpp)
set_target_properties(system_info PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(system_info PUBLIC cxx_std_17)
target_include_directories(system_info PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set(NODE_DEPENDENCIES
  ament_index_cpp::ament_index_cpp
  ${diagnostic_msgs_TARGETS}
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_library(system_monitor_component SHARED src/system_monitor_component.cpp)
target_link_libraries(system_monitor_component PRIVATE
  system_info ${NODE_DEPENDENCIES} rclcpp_components::component)
rclcpp_components_register_nodes(system_monitor_component "r5_monitor::SystemMonitor")

add_executable(system_monitor src/system_monitor_main.cpp)
target_link_libraries(system_monitor PRIVATE system_info ${NODE_DEPENDENCIES})

install(TARGETS system_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS system_monitor DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_system_info test/test_system_info.cpp)
  target_link_libraries(test_system_info system_info)
endif()

ament_package()

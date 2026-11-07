cmake_minimum_required(VERSION 3.8)
project(r5_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The BNO055 itself, without ROS, so it can be tested without hardware.
add_library(bno055_device STATIC src/bno055.cpp src/i2c_bus.cpp)
set_target_properties(bno055_device PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(bno055_device PUBLIC cxx_std_17)
target_include_directories(bno055_device PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set(BNO055_NODE_DEPENDENCIES
  ${example_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# The driver as a component, for loading into a container.
add_library(bno055_component SHARED src/bno055_component.cpp)
target_link_libraries(bno055_component bno055_device ${BNO055_NODE_DEPENDENCIES}
  rclcpp_components::component)
rclcpp_components_register_nodes(bno055_component "r5_sensors::Bno055Node")

# And on its own: ros2 run r5_sensors bno055.
add_executable(bno055 src/bno055_main.cpp)
target_link_libraries(bno055 bno055_device ${BNO055_NODE_DEPENDENCIES})

install(TARGETS bno055_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS bno055 DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bno055 test/test_bno055.cpp)
  target_link_libraries(test_bno055 bno055_device)
endif()

ament_package()

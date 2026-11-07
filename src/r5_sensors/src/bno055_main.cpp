// Runs the BNO055 driver on its own, so a missing IMU or a bad parameter is logged as an error
// rather than ending in an uncaught exception.

#include <exception>
#include <memory>

#include "r5_sensors/bno055_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<r5_sensors::Bno055Node>(rclcpp::NodeOptions()));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("bno055"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}

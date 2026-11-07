#include <memory>

#include "r5_monitor/system_monitor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<r5_monitor::SystemMonitor>());
  rclcpp::shutdown();
  return 0;
}

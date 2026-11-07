// /r5/ping: a service that answers at once, so the web app can measure its round trip to ROS and
// check the link is alive. It replaces rosapi's /rosapi/get_time, which was the only thing the
// dashboard used rosapi for.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace r5_dashboard
{

class Ping : public rclcpp::Node
{
public:
  explicit Ping(const rclcpp::NodeOptions & options)
  : Node("ping", options)
  {
    service_ = create_service<std_srvs::srv::Trigger>(
      "/r5/ping",
      [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = true;
      });
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

}  // namespace r5_dashboard

RCLCPP_COMPONENTS_REGISTER_NODE(r5_dashboard::Ping)

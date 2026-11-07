// sprayer_stub: stands in for the unfinished sprayer (spec 5.5).
//
// Accepts /r5/sprayer/enable (std_msgs/Bool) and /r5/sprayer/flow_setpoint (std_msgs/Float32,
// L/min) and logs what a real sprayer would do. Load it with the simulator, or on the robot until
// the sprayer hardware exists.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace r5_dashboard
{

class SprayerStub : public rclcpp::Node
{
public:
  explicit SprayerStub(const rclcpp::NodeOptions & options)
  : Node("sprayer_stub", options), last_(now())
  {
    // job_manager latches the enable command.
    const auto latched = rclcpp::QoS(1).transient_local();
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/r5/sprayer/enable", latched,
      [this](const std_msgs::msg::Bool & msg) {on_enable(msg.data);});
    flow_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/r5/sprayer/flow_setpoint", 10,
      [this](const std_msgs::msg::Float32 & msg) {on_flow(msg.data);});
    RCLCPP_INFO(get_logger(), "Sprayer stub running: commands are logged, nothing is sprayed.");
  }

private:
  void on_enable(bool on)
  {
    if (on != on_) {
      on_ = on;
      RCLCPP_INFO(get_logger(), "Sprayer %s (total %.2f L)", on_ ? "ON" : "OFF", litres_);
    }
  }

  // Adds up what the last setpoint would have sprayed since it arrived. A gap of 2 s or more means
  // the setpoints stopped, so it isn't counted.
  void on_flow(float flow_l_per_min)
  {
    const rclcpp::Time t = now();
    const double dt = (t - last_).seconds();
    last_ = t;
    if (on_ && dt > 0.0 && dt < 2.0) {
      litres_ += flow_ * dt / 60.0;
    }
    flow_ = flow_l_per_min;
  }

  bool on_ = false;
  double flow_ = 0.0;
  double litres_ = 0.0;
  rclcpp::Time last_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr flow_sub_;
};

}  // namespace r5_dashboard

RCLCPP_COMPONENTS_REGISTER_NODE(r5_dashboard::SprayerStub)

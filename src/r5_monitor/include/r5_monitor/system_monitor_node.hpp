#ifndef R5_MONITOR__SYSTEM_MONITOR_NODE_HPP_
#define R5_MONITOR__SYSTEM_MONITOR_NODE_HPP_

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "r5_monitor/system_info.hpp"

namespace r5_monitor
{

// Reports the computer temperature, WiFi signal and software versions for the dashboard:
// /r5/versions (latched, once), /r5/wifi_signal_dbm and /diagnostics, every second.
class SystemMonitor : public rclcpp::Node
{
public:
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

  explicit SystemMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("system_monitor", options),
    wifi_interface_(declare_parameter<std::string>("wifi_interface", "")),
    // A Raspberry Pi starts throttling at 80-85 °C.
    temp_warn_c_(declare_parameter("temperature_warning_c", 75.0)),
    temp_error_c_(declare_parameter("temperature_error_c", 80.0)),
    wifi_poor_dbm_(declare_parameter("wifi_poor_dbm", -80.0))
  {
    const auto repo_path = declare_parameter<std::string>("repo_path", "");

    const auto latched = rclcpp::QoS(1).transient_local();
    versions_pub_ = create_publisher<DiagnosticStatus>("/r5/versions", latched);
    wifi_pub_ = create_publisher<std_msgs::msg::Float32>("/r5/wifi_signal_dbm", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    const char * distro = std::getenv("ROS_DISTRO");
    versions_pub_->publish(
      status(
        "Versions", DiagnosticStatus::OK, "",
        {{"r5", git_version(repository_paths(repo_path))},
          {"ROS", distro ? distro : "unknown"}}));

    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {update();});
  }

private:
  using Values = std::vector<std::pair<std::string, std::string>>;

  static DiagnosticStatus status(
    const std::string & name, uint8_t level, const std::string & message,
    const Values & values = {})
  {
    DiagnosticStatus s;
    s.name = name;
    s.level = level;
    s.message = message;
    s.hardware_id = "r5";
    for (const auto & [key, value] : values) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      s.values.push_back(kv);
    }
    return s;
  }

  static std::string format(const char * fmt, double value)
  {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), fmt, value);
    return buffer;
  }

  // On the robot the workspace is the repository, so the installed package is inside it. The
  // working directory is a fallback for installs elsewhere.
  static std::vector<std::filesystem::path> repository_paths(const std::string & repo_path)
  {
    if (!repo_path.empty()) {
      return {repo_path};
    }
    std::vector<std::filesystem::path> paths;
    try {
      std::error_code ec;
      const auto prefix = std::filesystem::weakly_canonical(
        ament_index_cpp::get_package_prefix("r5_monitor"), ec);
      if (!ec) {
        paths.push_back(prefix);
      }
    } catch (const ament_index_cpp::PackageNotFoundError &) {
    }
    std::error_code ec;
    const auto cwd = std::filesystem::current_path(ec);
    if (!ec) {
      paths.push_back(cwd);
    }
    return paths;
  }

  void update()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    const auto temp = read_temperature_c("/sys/class/thermal");
    if (!temp) {
      array.status.push_back(
        status("r5: Computer temperature", DiagnosticStatus::WARN, "No temperature sensor"));
    } else {
      uint8_t level = DiagnosticStatus::OK;
      std::string message = format("%.0f °C", *temp);
      if (*temp >= temp_error_c_) {
        level = DiagnosticStatus::ERROR;
        message = "Overheating: " + message;
      } else if (*temp >= temp_warn_c_) {
        level = DiagnosticStatus::WARN;
        message = "Hot: " + message;
      }
      array.status.push_back(
        status(
          "r5: Computer temperature", level, message, {{"celsius", format("%.1f", *temp)}}));
    }

    const auto wifi = read_wifi("/proc/net/wireless", wifi_interface_);
    if (!wifi) {
      if (!warned_no_wifi_) {
        RCLCPP_WARN(get_logger(), "No wireless interface found, so no WiFi signal.");
        warned_no_wifi_ = true;
      }
    } else {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(wifi->dbm);
      wifi_pub_->publish(msg);
      const std::string dbm = format("%.0f", wifi->dbm);
      array.status.push_back(
        status(
          "r5: WiFi", wifi->dbm < wifi_poor_dbm_ ? DiagnosticStatus::WARN : DiagnosticStatus::OK,
          dbm + " dBm", {{"interface", wifi->interface}, {"dbm", dbm}}));
    }

    diagnostics_pub_->publish(array);
  }

  const std::string wifi_interface_;
  const double temp_warn_c_;
  const double temp_error_c_;
  const double wifi_poor_dbm_;
  bool warned_no_wifi_ = false;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr versions_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wifi_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace r5_monitor

#endif  // R5_MONITOR__SYSTEM_MONITOR_NODE_HPP_

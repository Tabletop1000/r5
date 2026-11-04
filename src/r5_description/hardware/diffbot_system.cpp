// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "r5/diffbot_system.hpp"
#include "serial_device.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r5
{

namespace
{
constexpr char FRONT_PORT[] = "/dev/ttyMotornodeFront";
constexpr char REAR_PORT[] = "/dev/ttyMotornodeRear";

// The motor nodes send 20 lines a second, so a gap longer than this means the link is down.
constexpr double LINK_TIMEOUT_S = 0.5;

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!motors_front_.init(FRONT_PORT, B115200))
  {
    RCLCPP_FATAL(get_logger(), "Failed to open front motor node.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!motors_rear_.init(REAR_PORT, B115200))
  {
    RCLCPP_FATAL(get_logger(), "Failed to open rear motor node.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  // Motor link health for /diagnostics. The controller manager spins this component's node.
  if (auto node = get_node())
  {
    diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());
    diagnostics_timer_ = node->create_wall_timer(
      std::chrono::seconds(1), [this]() { publish_diagnostics(); });
  }
  else
  {
    RCLCPP_WARN(get_logger(), "No component node, so motor link diagnostics are off.");
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_motors();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_motors();

  return hardware_interface::CallbackReturn::SUCCESS;
}

void DiffBotSystemHardware::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  const int64_t now_ns = steady_now_ns();

  const std::pair<const char *, const LinkStats *> links[] = {
    {FRONT_PORT, &front_link_}, {REAR_PORT, &rear_link_}};
  for (const auto & [port, link] : links)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    const bool front = link == &front_link_;
    status.name = front ? "r5: Motor node front" : "r5: Motor node rear";
    status.hardware_id = port;

    const int64_t last_ns = link->last_data_ns.load();
    const double age_s = last_ns == 0 ? -1.0 : static_cast<double>(now_ns - last_ns) * 1e-9;
    if (last_ns == 0)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "No data received";
    }
    else if (age_s > LINK_TIMEOUT_S)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "No data for " + std::to_string(static_cast<int>(age_s)) + " s";
    }
    else
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Link up";
    }

    auto value = [&status](const std::string & key, const std::string & v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = v;
      status.values.push_back(kv);
    };
    value("Port", port);
    value("Seconds since last data", last_ns == 0 ? "never" : std::to_string(age_s));
    value("Lines received", std::to_string(link->lines_received.load()));
    value("Write failures", std::to_string(link->write_failures.load()));
    array.status.push_back(status);
  }
  diagnostics_pub_->publish(array);
}

void DiffBotSystemHardware::stop_motors()
{
  const bool front_stopped = motors_front_.writeData(0.0f, 0.0f);
  const bool rear_stopped = motors_rear_.writeData(0.0f, 0.0f);
  if (!front_stopped || !rear_stopped)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send stop command to motor nodes.");
  }
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  float vel_front_right = 0.0;
  float vel_front_left = 0.0;
  float vel_rear_right = 0.0;
  float vel_rear_left = 0.0;
  float pos_front_right = 0.0;
  float pos_front_left = 0.0;
  float pos_rear_right = 0.0;
  float pos_rear_left = 0.0;

  if(motors_front_.readData((vel_front_right), (vel_front_left), (pos_front_right), (pos_front_left)))
  {
    front_link_.last_data_ns.store(steady_now_ns());
    front_link_.lines_received.fetch_add(1);
    set_state("front_right_wheel_joint/velocity",static_cast<double>(vel_front_right));
    set_state("front_left_wheel_joint/velocity",static_cast<double>(vel_front_left));
    set_state("front_right_wheel_joint/position", static_cast<double>(pos_front_right));
    set_state("front_left_wheel_joint/position", static_cast<double>(pos_front_left));
  }
  if(motors_rear_.readData((vel_rear_right), (vel_rear_left), (pos_rear_right), (pos_rear_left)))
  {
    rear_link_.last_data_ns.store(steady_now_ns());
    rear_link_.lines_received.fetch_add(1);
    set_state("rear_right_wheel_joint/velocity",static_cast<double>(vel_rear_right));
    set_state("rear_left_wheel_joint/velocity",static_cast<double>(vel_rear_left));
    set_state("rear_right_wheel_joint/position", static_cast<double>(pos_rear_right));
    set_state("rear_left_wheel_joint/position", static_cast<double>(pos_rear_left));
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type r5 ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  float vel_front_right = static_cast<float>(get_command("front_right_wheel_joint/velocity"));
  float vel_front_left = static_cast<float>(get_command("front_left_wheel_joint/velocity"));
  float vel_rear_right = static_cast<float>(get_command("rear_right_wheel_joint/velocity"));
  float vel_rear_left = static_cast<float>(get_command("rear_left_wheel_joint/velocity"));

  if (!motors_front_.writeData(vel_front_right,vel_front_left))
  {
    front_link_.write_failures.fetch_add(1);
  }
  if (!motors_rear_.writeData(vel_rear_right,vel_rear_left))
  {
    rear_link_.write_failures.fetch_add(1);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace r5

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  r5::DiffBotSystemHardware, hardware_interface::SystemInterface)
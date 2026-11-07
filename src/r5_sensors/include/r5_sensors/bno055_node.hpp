// ROS 2 driver for the Bosch BNO055 IMU over I2C. A C++ replacement for the Python `bno055`
// package (flynneva/bno055 0.5.0), with the same node name, parameters, topics and service, so
// r5_sensors/config/bno055_params.yaml and everything downstream work unchanged.
//
// Differences from the Python driver:
// - /bno055/imu's linear_acceleration includes gravity, as REP 145 and the simulator have it. The
//   Python driver sent the gravity-free linear acceleration there. It's still on /bno055/imu_raw
//   (raw accelerometer) and gravity alone on /bno055/grav.
// - Accelerations are in m/s². The Python driver set the chip to mg but divided by 100, so 1 g
//   read as 10.0.
// - One I2C transfer per sample, one timestamp for all of a sample's messages.
// - Temperature is signed, so it's right below 0 °C.
// - After about a second of failed reads it reconfigures the chip, once a second until it answers.
// - calibration_request returns to the configured operation mode rather than always NDOF.
// - Only connection_type "i2c". The robot doesn't use UART.

#ifndef R5_SENSORS__BNO055_NODE_HPP_
#define R5_SENSORS__BNO055_NODE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "example_interfaces/srv/trigger.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "r5_sensors/bno055.hpp"
#include "r5_sensors/i2c_bus.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"

namespace r5_sensors
{

namespace bno055_node_detail
{

constexpr int kChipIdAttempts = 10;  // The chip takes up to 650 ms to start after power-on.
constexpr std::chrono::milliseconds kChipIdRetry{100};
constexpr std::chrono::seconds kReconfigureInterval{1};

inline std::array<double, 9> diagonal(const std::vector<double> & v)
{
  return {v[0], 0.0, 0.0, 0.0, v[1], 0.0, 0.0, 0.0, v[2]};
}

inline geometry_msgs::msg::Vector3 to_msg(const bno055::Vec3 & v)
{
  geometry_msgs::msg::Vector3 m;
  m.x = v.x;
  m.y = v.y;
  m.z = v.z;
  return m;
}

}  // namespace bno055_node_detail

class Bno055Node : public rclcpp::Node
{
public:
  explicit Bno055Node(const rclcpp::NodeOptions & options)
  : Node("bno055", options)
  {
    const auto prefix = declare_parameter<std::string>("ros_topic_prefix", "bno055/");
    const auto connection_type = declare_parameter<std::string>("connection_type", "i2c");
    const auto i2c_bus = declare_parameter<int>("i2c_bus", 0);
    const auto i2c_addr = declare_parameter<int>("i2c_addr", 0x28);
    frame_id_ = declare_parameter<std::string>("frame_id", "bno055");
    const auto data_hz = declare_frequency("data_query_frequency", 10.0);
    const auto calib_hz = declare_frequency("calib_status_frequency", 0.1);
    const auto mode = declare_parameter<int>("operation_mode", 0x0C);
    const auto placement = declare_parameter<std::string>("placement_axis_remap", "P1");
    scale_.acc = declare_parameter<double>("acc_factor", 100.0);
    scale_.mag = declare_parameter<double>("mag_factor", 16000000.0);
    scale_.gyr = declare_parameter<double>("gyr_factor", 900.0);
    scale_.grav = declare_parameter<double>("grav_factor", 100.0);
    const auto set_offsets = declare_parameter<bool>("set_offsets", false);
    const auto offset_acc =
      declare_parameter<std::vector<int64_t>>("offset_acc", {0xFFEC, 0x00A5, 0xFFE8});
    const auto offset_mag =
      declare_parameter<std::vector<int64_t>>("offset_mag", {0xFFB4, 0xFE9E, 0x027D});
    const auto offset_gyr =
      declare_parameter<std::vector<int64_t>>("offset_gyr", {0x0002, 0xFFFF, 0xFFFF});
    const auto radius_acc = declare_parameter<int>("radius_acc", 0x3E8);
    const auto radius_mag = declare_parameter<int>("radius_mag", 0x0);
    accel_cov_ = bno055_node_detail::diagonal(variance("variance_acc", {0.017, 0.017, 0.017}));
    gyro_cov_ = bno055_node_detail::diagonal(variance("variance_angular_vel", {0.04, 0.04, 0.04}));
    orientation_cov_ = bno055_node_detail::diagonal(variance("variance_orientation", {0.0159, 0.0159, 0.0159}));
    mag_cov_ = bno055_node_detail::diagonal(variance("variance_mag", {0.0, 0.0, 0.0}));

    if (connection_type != "i2c") {
      throw std::invalid_argument(
              "connection_type is \"" + connection_type + "\"; this driver only supports \"i2c\"");
    }
    const auto axis_map = bno055::axis_map_for_placement(placement);
    if (!axis_map) {
      throw std::invalid_argument(
              "placement_axis_remap is \"" + placement + "\"; it must be P0 to P7");
    }
    if (data_hz <= 0.0 || calib_hz <= 0.0) {
      throw std::invalid_argument("data_query_frequency and calib_status_frequency must be > 0");
    }
    config_.operation_mode = static_cast<uint8_t>(mode);
    config_.axis_map = *axis_map;
    if (set_offsets) {
      bno055::Offsets o;
      o.accel = triple("offset_acc", offset_acc);
      o.mag = triple("offset_mag", offset_mag);
      o.gyro = triple("offset_gyr", offset_gyr);
      o.accel_radius = static_cast<uint16_t>(radius_acc);
      o.mag_radius = static_cast<uint16_t>(radius_mag);
      config_.offsets = o;
    }
    reconfigure_after_ = std::max(1, static_cast<int>(data_hz));

    const auto device = "/dev/i2c-" + std::to_string(i2c_bus);
    bus_ = std::make_unique<bno055::I2cBus>(device, static_cast<uint16_t>(i2c_addr));
    device_ = std::make_unique<bno055::Device>(*bus_);
    wait_for_chip(device, i2c_addr);
    configure();

    const rclcpp::QoS qos(10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(prefix + "imu_raw", qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(prefix + "imu", qos);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(prefix + "mag", qos);
    grav_pub_ = create_publisher<geometry_msgs::msg::Vector3>(prefix + "grav", qos);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(prefix + "temp", qos);
    calib_pub_ = create_publisher<std_msgs::msg::String>(prefix + "calib_status", qos);
    calib_srv_ = create_service<example_interfaces::srv::Trigger>(
      prefix + "calibration_request",
      [this](
        const std::shared_ptr<example_interfaces::srv::Trigger::Request>,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response) {
        on_calibration_request(*response);
      });

    // Callbacks run one at a time on this node's executor, so bus transfers never overlap.
    data_timer_ = create_wall_timer(period(data_hz), [this] {read_data();});
    calib_timer_ = create_wall_timer(period(calib_hz), [this] {read_calib_status();});
    RCLCPP_INFO(
      get_logger(), "BNO055 on %s at 0x%02X, mode 0x%02X, placement %s, %.0f Hz",
      device.c_str(), static_cast<unsigned>(i2c_addr), static_cast<unsigned>(mode),
      placement.c_str(), data_hz);
  }

private:
  // A rate in Hz, which the YAML may give as a whole number (100) or not (1.0), as the Python
  // driver accepted either.
  double declare_frequency(const std::string & name, double default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    const auto value = declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(value.get<int64_t>());
    }
    return value.get<double>();
  }

  std::vector<double> variance(const std::string & name, const std::vector<double> & default_value)
  {
    const auto v = declare_parameter<std::vector<double>>(name, default_value);
    if (v.size() != 3) {
      throw std::invalid_argument(name + " must have 3 values");
    }
    return v;
  }

  static std::array<uint16_t, 3> triple(const std::string & name, const std::vector<int64_t> & v)
  {
    if (v.size() != 3) {
      throw std::invalid_argument(name + " must have 3 values");
    }
    return {static_cast<uint16_t>(v[0]), static_cast<uint16_t>(v[1]), static_cast<uint16_t>(v[2])};
  }

  static std::chrono::nanoseconds period(double hz)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
  }

  void wait_for_chip(const std::string & device, int address)
  {
    std::string last_error = "wrong chip ID";
    for (int attempt = 0; attempt < bno055_node_detail::kChipIdAttempts; ++attempt) {
      try {
        if (device_->chip_id_ok()) {
          return;
        }
        last_error = "wrong chip ID";
      } catch (const bno055::BusError & e) {
        last_error = e.what();
      }
      rclcpp::sleep_for(bno055_node_detail::kChipIdRetry);
    }
    char addr[8];
    std::snprintf(addr, sizeof(addr), "0x%02X", address);
    throw std::runtime_error(
            "No BNO055 on " + device + " at " + addr + " (" + last_error + ")");
  }

  void configure()
  {
    const auto before = device_->configure(config_);
    RCLCPP_INFO(get_logger(), "Offsets in the chip: %s", bno055::offsets_text(before).c_str());
    if (config_.offsets) {
      RCLCPP_INFO(get_logger(), "Wrote offsets: %s", bno055::offsets_text(*config_.offsets).c_str());
    }
  }

  void read_data()
  {
    // A failed configure can leave the chip in config mode, where reads still work but the data
    // stops changing. Publish nothing until it's configured again.
    if (!configured_) {
      try_reconfigure();
      if (!configured_) {
        return;
      }
    }
    bno055::Sample sample;
    try {
      sample = bno055::decode_sample(device_->read_sample(), scale_);
    } catch (const bno055::BusError & e) {
      on_read_failed(e.what());
      return;
    }
    if (failures_ >= reconfigure_after_) {
      RCLCPP_INFO(get_logger(), "The IMU is answering again after %d failed reads", failures_);
    }
    failures_ = 0;
    publish(sample, now());
  }

  void on_read_failed(const std::string & error)
  {
    ++failures_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, 5000, "Reading the IMU failed (%d in a row): %s",
      failures_, error.c_str());
    if (failures_ >= reconfigure_after_) {
      configured_ = false;
      try_reconfigure();
    }
  }

  // At most once per kReconfigureInterval.
  void try_reconfigure()
  {
    const auto t = steady_clock_.now();
    if (t - last_reconfigure_ < bno055_node_detail::kReconfigureInterval) {
      return;
    }
    last_reconfigure_ = t;
    try {
      configure();
      configured_ = true;
      RCLCPP_INFO(get_logger(), "Reconfigured the IMU");
    } catch (const bno055::BusError & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 5000, "Reconfiguring the IMU failed: %s", e.what());
    }
  }

  void publish(const bno055::Sample & s, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Imu raw;
    raw.header.stamp = stamp;
    raw.header.frame_id = frame_id_;
    raw.orientation_covariance = orientation_cov_;
    raw.linear_acceleration = bno055_node_detail::to_msg(s.accel);
    raw.linear_acceleration_covariance = accel_cov_;
    raw.angular_velocity = bno055_node_detail::to_msg(s.gyro);
    raw.angular_velocity_covariance = gyro_cov_;
    imu_raw_pub_->publish(raw);

    // The chip reports an all-zero quaternion until fusion starts. The Python driver skipped the
    // fused messages then too.
    if (s.orientation) {
      auto imu = raw;
      imu.orientation.w = s.orientation->w;
      imu.orientation.x = s.orientation->x;
      imu.orientation.y = s.orientation->y;
      imu.orientation.z = s.orientation->z;
      imu_pub_->publish(imu);
    }

    sensor_msgs::msg::MagneticField mag;
    mag.header = raw.header;
    mag.magnetic_field = bno055_node_detail::to_msg(s.mag);
    mag.magnetic_field_covariance = mag_cov_;
    mag_pub_->publish(mag);

    grav_pub_->publish(bno055_node_detail::to_msg(s.gravity));

    sensor_msgs::msg::Temperature temp;
    temp.header = raw.header;
    temp.temperature = s.temperature_c;
    temp_pub_->publish(temp);
  }

  void read_calib_status()
  {
    try {
      std_msgs::msg::String msg;
      msg.data = bno055::calib_status_json(bno055::decode_calib_status(device_->read_calib_status()));
      calib_pub_->publish(msg);
    } catch (const bno055::BusError & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 5000, "Reading the calibration status failed: %s", e.what());
    }
  }

  void on_calibration_request(example_interfaces::srv::Trigger::Response & response)
  {
    try {
      device_->set_mode(0x00);
      const auto offsets = device_->read_offsets();
      device_->set_mode(config_.operation_mode);
      response.success = true;
      response.message = bno055::offsets_text(offsets);
    } catch (const bno055::BusError & e) {
      // It may still be in config mode.
      configured_ = false;
      response.success = false;
      response.message = e.what();
    }
  }

  std::string frame_id_;
  bno055::Scale scale_;
  bno055::Config config_;
  std::array<double, 9> accel_cov_{};
  std::array<double, 9> gyro_cov_{};
  std::array<double, 9> orientation_cov_{};
  std::array<double, 9> mag_cov_{};

  std::unique_ptr<bno055::I2cBus> bus_;
  std::unique_ptr<bno055::Device> device_;
  bool configured_ = true;  // The constructor configures it or throws.
  int failures_ = 0;
  int reconfigure_after_ = 1;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_reconfigure_{0, 0, RCL_STEADY_TIME};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr grav_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calib_pub_;
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr calib_srv_;
  rclcpp::TimerBase::SharedPtr data_timer_;
  rclcpp::TimerBase::SharedPtr calib_timer_;
};

}  // namespace r5_sensors

#endif  // R5_SENSORS__BNO055_NODE_HPP_

#ifndef R5_SENSORS__BNO055_HPP_
#define R5_SENSORS__BNO055_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>

// The BNO055 IMU, without ROS: configuring it, reading its data and decoding it. The ROS node is in
// bno055_node.cpp. The bus is an interface so the tests can use a fake one.
namespace r5_sensors::bno055
{

class BusError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Reads and writes runs of consecutive registers. Throws BusError on failure.
class RegisterBus
{
public:
  virtual ~RegisterBus() = default;
  virtual void read(uint8_t reg, uint8_t * data, std::size_t length) = 0;
  virtual void write(uint8_t reg, const uint8_t * data, std::size_t length) = 0;
};

// Raw 16-bit register values, as the datasheet gives them. Negative offsets are two's complement,
// for example 0xFFEC for -20.
struct Offsets
{
  std::array<uint16_t, 3> accel{};
  std::array<uint16_t, 3> mag{};
  std::array<uint16_t, 3> gyro{};
  uint16_t accel_radius = 0;
  uint16_t mag_radius = 0;
};

struct Config
{
  uint8_t operation_mode = 0x0C;  // NDOF: all sensors, absolute orientation.
  std::array<uint8_t, 2> axis_map{0x24, 0x00};  // Placement P1, the chip's default.
  std::optional<Offsets> offsets;  // Written while configuring, when set.
};

// Divisors from register counts to SI units, as the driver parameters of the same names.
struct Scale
{
  double acc = 100.0;  // To m/s².
  double mag = 16000000.0;  // To tesla.
  double gyr = 900.0;  // To rad/s.
  double grav = 100.0;  // To m/s².
};

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// Length of the sensor data block that starts at reg::kAccelDataX.
constexpr std::size_t kSampleLength = 45;
using SampleBytes = std::array<uint8_t, kSampleLength>;

struct Sample
{
  Vec3 accel;  // Including gravity.
  Vec3 mag;
  Vec3 gyro;
  std::optional<Quaternion> orientation;  // Normalised. Empty if the chip reported all zeros.
  Vec3 linear_accel;  // Gravity removed.
  Vec3 gravity;
  int temperature_c = 0;
};

struct CalibStatus
{
  int sys = 0;
  int gyro = 0;
  int accel = 0;
  int mag = 0;
};

// AXIS_MAP_CONFIG and AXIS_MAP_SIGN for a placement P0 to P7 (datasheet section 3.4), or empty if
// the name isn't one of them.
std::optional<std::array<uint8_t, 2>> axis_map_for_placement(const std::string & placement);

Sample decode_sample(const SampleBytes & bytes, const Scale & scale);
CalibStatus decode_calib_status(uint8_t value);
// The same JSON the Python driver published, which the dashboard parses.
std::string calib_status_json(const CalibStatus & status);
// Offsets in the form the Python driver's calibration_request service returned.
std::string offsets_text(const Offsets & offsets);

class Device
{
public:
  using Sleep = std::function<void (std::chrono::milliseconds)>;

  // `sleep` waits for mode changes to take effect; the tests pass one that returns at once.
  explicit Device(RegisterBus & bus, Sleep sleep = {});

  bool chip_id_ok();
  // Returns the offsets the chip held before any were written, to log them.
  Offsets configure(const Config & config);
  void set_mode(uint8_t mode);
  SampleBytes read_sample();
  uint8_t read_calib_status();
  // Only valid in config mode.
  Offsets read_offsets();
  // Switches to config mode first. Leaves the chip in config mode.
  void write_offsets(const Offsets & offsets);

private:
  void write_byte(uint8_t reg, uint8_t value);

  RegisterBus & bus_;
  Sleep sleep_;
};

}  // namespace r5_sensors::bno055

#endif  // R5_SENSORS__BNO055_HPP_

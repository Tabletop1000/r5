#include "r5_sensors/bno055.hpp"

#include <cmath>
#include <map>
#include <sstream>
#include <thread>
#include <utility>

#include "r5_sensors/bno055_registers.hpp"

namespace r5_sensors::bno055
{

namespace
{

// Waits after a mode change (datasheet table 3-6: 19 ms into config mode, 7 ms out of it).
constexpr std::chrono::milliseconds kToConfigDelay{25};
constexpr std::chrono::milliseconds kFromConfigDelay{10};

// Offsets and radii, 0x55 to 0x6A.
constexpr std::size_t kOffsetsLength = 22;

int16_t s16(const uint8_t * p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0] | (p[1] << 8)));
}

uint16_t u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

Vec3 vec3(const uint8_t * p, double divisor)
{
  return {s16(p) / divisor, s16(p + 2) / divisor, s16(p + 4) / divisor};
}

std::string xyz_text(const std::array<uint16_t, 3> & v)
{
  std::ostringstream out;
  out << "{'x': " << v[0] << ", 'y': " << v[1] << ", 'z': " << v[2] << "}";
  return out.str();
}

}  // namespace

std::optional<std::array<uint8_t, 2>> axis_map_for_placement(const std::string & placement)
{
  // Datasheet section 3.4. The Python driver had P5's sign as 0x02, the same as P3.
  static const std::map<std::string, std::array<uint8_t, 2>> kPlacements = {
    {"P0", {0x21, 0x04}}, {"P1", {0x24, 0x00}}, {"P2", {0x24, 0x06}}, {"P3", {0x21, 0x02}},
    {"P4", {0x24, 0x03}}, {"P5", {0x21, 0x01}}, {"P6", {0x21, 0x07}}, {"P7", {0x24, 0x05}},
  };
  const auto it = kPlacements.find(placement);
  if (it == kPlacements.end()) {
    return std::nullopt;
  }
  return it->second;
}

Sample decode_sample(const SampleBytes & b, const Scale & scale)
{
  // Offsets into the block that starts at 0x08.
  Sample s;
  s.accel = vec3(&b[0x08 - 0x08], scale.acc);
  s.mag = vec3(&b[0x0E - 0x08], scale.mag);
  s.gyro = vec3(&b[0x14 - 0x08], scale.gyr);
  const double w = s16(&b[0x20 - 0x08]);
  const double x = s16(&b[0x22 - 0x08]);
  const double y = s16(&b[0x24 - 0x08]);
  const double z = s16(&b[0x26 - 0x08]);
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (norm > 0.0) {
    s.orientation = Quaternion{w / norm, x / norm, y / norm, z / norm};
  }
  s.linear_accel = vec3(&b[0x28 - 0x08], scale.acc);
  s.gravity = vec3(&b[0x2E - 0x08], scale.grav);
  s.temperature_c = static_cast<int8_t>(b[0x34 - 0x08]);
  return s;
}

CalibStatus decode_calib_status(uint8_t v)
{
  return {(v >> 6) & 0x03, (v >> 4) & 0x03, (v >> 2) & 0x03, v & 0x03};
}

std::string calib_status_json(const CalibStatus & c)
{
  std::ostringstream out;
  out << "{\"sys\": " << c.sys << ", \"gyro\": " << c.gyro << ", \"accel\": " << c.accel <<
    ", \"mag\": " << c.mag << "}";
  return out.str();
}

std::string offsets_text(const Offsets & o)
{
  std::ostringstream out;
  out << "{'accel_offset': " << xyz_text(o.accel) << ", 'accel_radius': " << o.accel_radius <<
    ", 'mag_offset': " << xyz_text(o.mag) << ", 'mag_radius': " << o.mag_radius <<
    ", 'gyro_offset': " << xyz_text(o.gyro) << "}";
  return out.str();
}

Device::Device(RegisterBus & bus, Sleep sleep)
: bus_(bus), sleep_(std::move(sleep))
{
  if (!sleep_) {
    sleep_ = [](std::chrono::milliseconds d) {std::this_thread::sleep_for(d);};
  }
}

bool Device::chip_id_ok()
{
  uint8_t id = 0;
  bus_.read(reg::kChipId, &id, 1);
  return id == reg::kChipIdValue;
}

Offsets Device::configure(const Config & config)
{
  set_mode(reg::kModeConfig);
  write_byte(reg::kPwrMode, reg::kPowerNormal);
  write_byte(reg::kPageId, 0x00);
  write_byte(reg::kSysTrigger, 0x00);
  write_byte(reg::kUnitSel, reg::kUnitSelValue);
  bus_.write(reg::kAxisMapConfig, config.axis_map.data(), config.axis_map.size());
  const Offsets before = read_offsets();
  if (config.offsets) {
    write_offsets(*config.offsets);
  }
  set_mode(config.operation_mode);
  return before;
}

void Device::set_mode(uint8_t mode)
{
  write_byte(reg::kOprMode, mode);
  sleep_(mode == reg::kModeConfig ? kToConfigDelay : kFromConfigDelay);
}

SampleBytes Device::read_sample()
{
  SampleBytes bytes{};
  bus_.read(reg::kAccelDataX, bytes.data(), bytes.size());
  return bytes;
}

uint8_t Device::read_calib_status()
{
  uint8_t value = 0;
  bus_.read(reg::kCalibStat, &value, 1);
  return value;
}

Offsets Device::read_offsets()
{
  std::array<uint8_t, kOffsetsLength> b{};
  bus_.read(reg::kAccelOffsetX, b.data(), b.size());
  Offsets o;
  for (std::size_t i = 0; i < 3; ++i) {
    o.accel[i] = u16(&b[reg::kAccelOffsetX - reg::kAccelOffsetX + 2 * i]);
    o.mag[i] = u16(&b[reg::kMagOffsetX - reg::kAccelOffsetX + 2 * i]);
    o.gyro[i] = u16(&b[reg::kGyroOffsetX - reg::kAccelOffsetX + 2 * i]);
  }
  o.accel_radius = u16(&b[reg::kAccelRadius - reg::kAccelOffsetX]);
  o.mag_radius = u16(&b[reg::kMagRadius - reg::kAccelOffsetX]);
  return o;
}

void Device::write_offsets(const Offsets & o)
{
  set_mode(reg::kModeConfig);
  // One register at a time, as the Python driver found writing them together didn't work.
  const auto put = [this](uint8_t reg, uint16_t value) {
      write_byte(reg, static_cast<uint8_t>(value & 0xFF));
      write_byte(static_cast<uint8_t>(reg + 1), static_cast<uint8_t>(value >> 8));
    };
  for (uint8_t i = 0; i < 3; ++i) {
    put(static_cast<uint8_t>(reg::kAccelOffsetX + 2 * i), o.accel[i]);
  }
  put(reg::kAccelRadius, o.accel_radius);
  for (uint8_t i = 0; i < 3; ++i) {
    put(static_cast<uint8_t>(reg::kMagOffsetX + 2 * i), o.mag[i]);
  }
  put(reg::kMagRadius, o.mag_radius);
  for (uint8_t i = 0; i < 3; ++i) {
    put(static_cast<uint8_t>(reg::kGyroOffsetX + 2 * i), o.gyro[i]);
  }
}

void Device::write_byte(uint8_t reg, uint8_t value)
{
  bus_.write(reg, &value, 1);
}

}  // namespace r5_sensors::bno055

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

#include "r5_sensors/bno055.hpp"
#include "r5_sensors/bno055_registers.hpp"

namespace b = r5_sensors::bno055;
namespace reg = r5_sensors::bno055::reg;

namespace
{

// The chip's registers, plus a log of every write.
class FakeBus : public b::RegisterBus
{
public:
  void read(uint8_t r, uint8_t * data, std::size_t length) override
  {
    if (fail) {
      throw b::BusError("no answer");
    }
    std::memcpy(data, &regs[r], length);
    reads.push_back({r, length});
  }

  void write(uint8_t r, const uint8_t * data, std::size_t length) override
  {
    if (fail) {
      throw b::BusError("no answer");
    }
    std::memcpy(&regs[r], data, length);
    writes.push_back({r, std::vector<uint8_t>(data, data + length)});
  }

  void put_s16(uint8_t r, int16_t v)
  {
    regs[r] = static_cast<uint8_t>(v & 0xFF);
    regs[r + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }

  std::array<uint8_t, 256> regs{};
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> writes;
  std::vector<std::pair<uint8_t, std::size_t>> reads;
  bool fail = false;
};

b::Device device(FakeBus & bus)
{
  return b::Device(bus, [](std::chrono::milliseconds) {});
}

}  // namespace

TEST(Bno055, ChecksTheChipId)
{
  FakeBus bus;
  auto d = device(bus);
  EXPECT_FALSE(d.chip_id_ok());
  bus.regs[reg::kChipId] = 0xA0;
  EXPECT_TRUE(d.chip_id_ok());
}

// As the Python driver, plus the temperature source.
TEST(Bno055, ConfiguresInTheSameOrderAsThePythonDriver)
{
  FakeBus bus;
  auto d = device(bus);
  b::Config config;
  config.operation_mode = 0x0C;
  config.axis_map = *b::axis_map_for_placement("P6");
  d.configure(config);

  using W = std::pair<uint8_t, std::vector<uint8_t>>;
  const std::vector<W> expected = {
    {reg::kOprMode, {0x00}},
    {reg::kPwrMode, {0x00}},
    {reg::kPageId, {0x00}},
    {reg::kSysTrigger, {0x00}},
    {reg::kUnitSel, {0x82}},
    {reg::kTempSource, {0x00}},
    {reg::kAxisMapConfig, {0x21, 0x07}},
    {reg::kOprMode, {0x0C}},
  };
  EXPECT_EQ(bus.writes, expected);
}

TEST(Bno055, WritesOffsetsOneByteAtATimeWhenAsked)
{
  FakeBus bus;
  auto d = device(bus);
  b::Config config;
  b::Offsets o;
  o.accel = {0xFFEC, 0x00A5, 0xFFE8};
  o.mag = {0xFFB4, 0xFE9E, 0x027D};
  o.gyro = {0x0002, 0xFFFF, 0xFFFF};
  o.accel_radius = 0x3E8;
  o.mag_radius = 0x1E0;
  config.offsets = o;
  d.configure(config);

  for (const auto & w : bus.writes) {
    EXPECT_EQ(w.second.size(), w.first == reg::kAxisMapConfig ? 2u : 1u);
  }
  EXPECT_EQ(bus.regs[0x55], 0xEC);
  EXPECT_EQ(bus.regs[0x56], 0xFF);
  EXPECT_EQ(bus.regs[0x67], 0xE8);
  EXPECT_EQ(bus.regs[0x68], 0x03);
  EXPECT_EQ(bus.writes.back(), (std::pair<uint8_t, std::vector<uint8_t>>{reg::kOprMode, {0x0C}}));
  // Read back the same way.
  const auto back = d.read_offsets();
  EXPECT_EQ(back.accel, o.accel);
  EXPECT_EQ(back.mag, o.mag);
  EXPECT_EQ(back.gyro, o.gyro);
  EXPECT_EQ(back.accel_radius, o.accel_radius);
  EXPECT_EQ(back.mag_radius, o.mag_radius);
}

TEST(Bno055, ReadsAWholeSampleInOneTransfer)
{
  FakeBus bus;
  auto d = device(bus);
  d.read_sample();
  ASSERT_EQ(bus.reads.size(), 1u);
  EXPECT_EQ(bus.reads[0], (std::pair<uint8_t, std::size_t>{0x08, 45}));
}

TEST(Bno055, DecodesASample)
{
  FakeBus bus;
  bus.put_s16(0x08, 12);  // Accelerometer, 0.01 m/s² per count.
  bus.put_s16(0x0A, -34);
  bus.put_s16(0x0C, 981);
  bus.put_s16(0x0E, 320);  // Magnetometer, 1/16 µT per count.
  bus.put_s16(0x14, 900);  // Gyroscope, 1/900 rad/s per count.
  bus.put_s16(0x16, -450);
  bus.put_s16(0x20, 1 << 14);  // Quaternion w = 1, x = 1: normalised to 1/√2 each.
  bus.put_s16(0x22, 1 << 14);
  bus.put_s16(0x28, 5);  // Linear acceleration.
  bus.put_s16(0x32, 980);  // Gravity z.
  bus.regs[0x34] = static_cast<uint8_t>(-5);  // Temperature, signed.
  auto d = device(bus);

  const auto s = b::decode_sample(d.read_sample(), b::Scale{});
  EXPECT_DOUBLE_EQ(s.accel.x, 0.12);
  EXPECT_DOUBLE_EQ(s.accel.y, -0.34);
  EXPECT_DOUBLE_EQ(s.accel.z, 9.81);
  EXPECT_DOUBLE_EQ(s.mag.x, 20e-6);
  EXPECT_DOUBLE_EQ(s.gyro.x, 1.0);
  EXPECT_DOUBLE_EQ(s.gyro.y, -0.5);
  ASSERT_TRUE(s.orientation.has_value());
  EXPECT_NEAR(s.orientation->w, 1.0 / std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(s.orientation->x, 1.0 / std::sqrt(2.0), 1e-12);
  EXPECT_DOUBLE_EQ(s.orientation->y, 0.0);
  EXPECT_DOUBLE_EQ(s.linear_accel.x, 0.05);
  EXPECT_DOUBLE_EQ(s.gravity.z, 9.8);
  EXPECT_EQ(s.temperature_c, -5);
}

TEST(Bno055, AnAllZeroQuaternionHasNoOrientation)
{
  b::SampleBytes bytes{};
  EXPECT_FALSE(b::decode_sample(bytes, b::Scale{}).orientation.has_value());
}

TEST(Bno055, CalibrationStatusAsThePythonDriverSentIt)
{
  const auto c = b::decode_calib_status(0b11100100);
  EXPECT_EQ(c.sys, 3);
  EXPECT_EQ(c.gyro, 2);
  EXPECT_EQ(c.accel, 1);
  EXPECT_EQ(c.mag, 0);
  EXPECT_EQ(b::calib_status_json(c), "{\"sys\": 3, \"gyro\": 2, \"accel\": 1, \"mag\": 0}");
}

TEST(Bno055, Placements)
{
  EXPECT_EQ(*b::axis_map_for_placement("P1"), (std::array<uint8_t, 2>{0x24, 0x00}));
  EXPECT_EQ(*b::axis_map_for_placement("P5"), (std::array<uint8_t, 2>{0x21, 0x01}));
  EXPECT_FALSE(b::axis_map_for_placement("P8").has_value());
  EXPECT_FALSE(b::axis_map_for_placement("p6").has_value());
}

TEST(Bno055, BusErrorsReachTheCaller)
{
  FakeBus bus;
  bus.fail = true;
  auto d = device(bus);
  EXPECT_THROW(d.read_sample(), b::BusError);
  EXPECT_THROW(d.configure(b::Config{}), b::BusError);
}

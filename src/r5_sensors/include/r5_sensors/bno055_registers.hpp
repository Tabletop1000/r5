#ifndef R5_SENSORS__BNO055_REGISTERS_HPP_
#define R5_SENSORS__BNO055_REGISTERS_HPP_

#include <cstdint>

// The BNO055 registers this driver uses, all on register page 0. See the Bosch BNO055 datasheet,
// section 4.2.
namespace r5_sensors::bno055::reg
{

constexpr uint8_t kChipId = 0x00;
constexpr uint8_t kPageId = 0x07;
// The sensor data block, read in one transfer: accelerometer, magnetometer, gyroscope, Euler
// angles, quaternion, linear acceleration, gravity, then temperature.
constexpr uint8_t kAccelDataX = 0x08;
constexpr uint8_t kCalibStat = 0x35;
constexpr uint8_t kUnitSel = 0x3B;
constexpr uint8_t kOprMode = 0x3D;
constexpr uint8_t kPwrMode = 0x3E;
constexpr uint8_t kSysTrigger = 0x3F;
constexpr uint8_t kAxisMapConfig = 0x41;  // Followed by kAxisMapSign at 0x42.
// Calibration offsets and radii, each a little-endian 16-bit value.
constexpr uint8_t kAccelOffsetX = 0x55;
constexpr uint8_t kMagOffsetX = 0x5B;
constexpr uint8_t kGyroOffsetX = 0x61;
constexpr uint8_t kAccelRadius = 0x67;
constexpr uint8_t kMagRadius = 0x69;

constexpr uint8_t kChipIdValue = 0xA0;
constexpr uint8_t kModeConfig = 0x00;
constexpr uint8_t kPowerNormal = 0x00;

// Accelerometer and gravity in m/s² (1 LSB = 0.01 m/s²), gyroscope in rad/s (1 LSB = 1/900 rad/s),
// Euler angles in degrees, temperature in °C, Android orientation convention.
constexpr uint8_t kUnitSelValue = 0x82;

}  // namespace r5_sensors::bno055::reg

#endif  // R5_SENSORS__BNO055_REGISTERS_HPP_

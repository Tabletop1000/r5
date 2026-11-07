#ifndef R5_SENSORS__I2C_BUS_HPP_
#define R5_SENSORS__I2C_BUS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "r5_sensors/bno055.hpp"

namespace r5_sensors::bno055
{

// A device on a Linux I2C bus (/dev/i2c-N). Each read is one combined transfer: the register
// address, a repeated start, then all the bytes, so a whole sample arrives in one go.
class I2cBus : public RegisterBus
{
public:
  // Throws BusError if the bus can't be opened.
  I2cBus(const std::string & device, uint16_t address);
  ~I2cBus() override;
  I2cBus(const I2cBus &) = delete;
  I2cBus & operator=(const I2cBus &) = delete;

  void read(uint8_t reg, uint8_t * data, std::size_t length) override;
  void write(uint8_t reg, const uint8_t * data, std::size_t length) override;

private:
  std::string device_;
  uint16_t address_;
  int fd_;
};

}  // namespace r5_sensors::bno055

#endif  // R5_SENSORS__I2C_BUS_HPP_

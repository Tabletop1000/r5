#include "r5_sensors/i2c_bus.hpp"

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

namespace r5_sensors::bno055
{

namespace
{

std::string error_text(const std::string & what)
{
  return what + ": " + std::strerror(errno);
}

}  // namespace

I2cBus::I2cBus(const std::string & device, uint16_t address)
: device_(device), address_(address), fd_(::open(device.c_str(), O_RDWR | O_CLOEXEC))
{
  if (fd_ < 0) {
    throw BusError(error_text("Can't open " + device_));
  }
}

I2cBus::~I2cBus()
{
  ::close(fd_);
}

void I2cBus::read(uint8_t reg, uint8_t * data, std::size_t length)
{
  i2c_msg msgs[2] = {};
  msgs[0].addr = address_;
  msgs[0].len = 1;
  msgs[0].buf = &reg;
  msgs[1].addr = address_;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = static_cast<uint16_t>(length);
  msgs[1].buf = data;
  i2c_rdwr_ioctl_data transfer{msgs, 2};
  if (::ioctl(fd_, I2C_RDWR, &transfer) != 2) {
    throw BusError(error_text("Reading " + device_));
  }
}

void I2cBus::write(uint8_t reg, const uint8_t * data, std::size_t length)
{
  std::vector<uint8_t> buffer(length + 1);
  buffer[0] = reg;
  std::memcpy(buffer.data() + 1, data, length);
  i2c_msg msg = {};
  msg.addr = address_;
  msg.len = static_cast<uint16_t>(buffer.size());
  msg.buf = buffer.data();
  i2c_rdwr_ioctl_data transfer{&msg, 1};
  if (::ioctl(fd_, I2C_RDWR, &transfer) != 1) {
    throw BusError(error_text("Writing " + device_));
  }
}

}  // namespace r5_sensors::bno055

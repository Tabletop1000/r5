#include "serial_device.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <algorithm>


SerialDevice::SerialDevice(const std::string& port, speed_t baud)
    : fd(-1), portName(port), baudRate(baud) {}

SerialDevice::~SerialDevice() {
    if (fd != -1) close(fd);
}

bool SerialDevice::init() {
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening serial port " << portName << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting termios: " << strerror(errno) << std::endl;
        return false;
    }

    // Configure serial settings
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // Disable break processing
    tty.c_lflag = 0;                            // No signaling chars, no echo
    tty.c_oflag = 0;                            // No remapping, no delays
    tty.c_cc[VMIN]  = 0;                        // Non-blocking read
    tty.c_cc[VTIME] = 10;                       // 1.0 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // No flow control
    tty.c_cflag |= (CLOCAL | CREAD);            // Enable receiver
    tty.c_cflag &= ~(PARENB | PARODD);          // No parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // No RTS/CTS

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool SerialDevice::writeData(float val1, float val2) {
    if (fd == -1) return false;
    std::ostringstream oss;
    oss << val1 << ":" << val2 << "\n";  // Append newline as message terminator
    std::string msg = oss.str();

    ssize_t bytesWritten = write(fd, msg.c_str(), msg.size());
    if (bytesWritten < 0) {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool SerialDevice::readData(float& val1, float& val2) {
    if (fd == -1) return false;

    char buf[256];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    if (n <= 0) return false;

    buf[n] = '\0';
    std::string data(buf);

    // Parse "<float>:<float>"
    std::replace(data.begin(), data.end(), '\n', '\0');
    size_t sep = data.find(':');
    if (sep == std::string::npos) return false;

    try {
        val1 = std::stof(data.substr(0, sep));
        val2 = std::stof(data.substr(sep + 1));
    } catch (...) {
        return false;
    }

    return true;
}

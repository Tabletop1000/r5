#include "serial_device.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>

namespace {
// Longest run of bytes kept while waiting for a newline.
constexpr size_t MAX_RX_BUFFER = 1024;
}


SerialDevice::SerialDevice()
    : fd(-1) {}

SerialDevice::~SerialDevice() {
    if (fd != -1) close(fd);
}

bool SerialDevice::init(const std::string& port, speed_t baud) {
    portName = port;
    baudRate = baud;
    rxBuffer.clear();
    if (fd != -1) close(fd);
    // O_NONBLOCK: this is called from the control loop, which must never wait on the port
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);    // No CR/NL translation on input
    tty.c_lflag = 0;                            // No signaling chars, no echo
    tty.c_oflag = 0;                            // No remapping, no delays
    tty.c_cc[VMIN]  = 0;                        // Non-blocking read
    tty.c_cc[VTIME] = 0;                        // No read timeout

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

bool SerialDevice::readData(float& val1, float& val2, float& val3, float& val4) {
    if (fd == -1) return false;

    // Drain everything waiting on the port
    char buf[256];
    ssize_t n;
    while ((n = read(fd, buf, sizeof(buf))) > 0) {
        rxBuffer.append(buf, n);
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        std::cerr << "Error reading from serial port " << portName << ": " << strerror(errno) << std::endl;
        return false;
    }

    size_t lastNewline = rxBuffer.rfind('\n');
    if (lastNewline == std::string::npos) {
        if (rxBuffer.size() > MAX_RX_BUFFER) rxBuffer.clear();
        return false;
    }

    // Only complete lines are parsed; anything after the last newline waits for the next call
    std::istringstream lines(rxBuffer.substr(0, lastNewline));
    rxBuffer.erase(0, lastNewline + 1);

    // Parse "<float>:<float>:<float>:<float>", keeping the newest valid line
    bool parsed = false;
    std::string line;
    while (std::getline(lines, line)) {
        float v1, v2, v3, v4;
        if (sscanf(line.c_str(), "%f:%f:%f:%f", &v1, &v2, &v3, &v4) == 4) {
            val1 = v1;
            val2 = v2;
            val3 = v3;
            val4 = v4;
            parsed = true;
        }
    }

    return parsed;
}

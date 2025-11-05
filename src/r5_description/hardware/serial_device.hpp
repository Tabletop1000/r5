#ifndef SERIALDEVICE_HPP
#define SERIALDEVICE_HPP

#include <string>
#include <termios.h>

class SerialDevice {
private:
    int fd;                 // File descriptor for the serial port
    std::string portName;   // e.g., "/dev/ttyUSB0"
    speed_t baudRate;       // e.g., B115200

public:
    SerialDevice(const std::string& port = "/dev/ttyUSB0", speed_t baud = B115200);
    ~SerialDevice();

    bool init();
    bool writeData(float val1, float val2);
    bool readData(float& val1, float& val2);
};

#endif // SERIALDEVICE_HPP

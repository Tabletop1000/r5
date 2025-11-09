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
    SerialDevice();
    ~SerialDevice();

    bool init(const std::string& port, speed_t baud);
    bool writeData(float val1, float val2);
    bool readData(float& val1, float& val2, float& val3, float& val4);
};

#endif // SERIALDEVICE_HPP

#include "serial_device.hpp"
#include <iostream>

using namespace std;

int main()
{
    SerialDevice motor;
    motor.init("/dev/ttyACM0",B115200);
    float v1,v2,p1,p2;
    while(1)
    {

        motor.readData(v1,v2,p1,p2);
        cout << "V1: " << v1 << " V2: " << v2 << "P1: " << p1 << "P2: " << p2 <<endl;
    }
    return 0;
}
    


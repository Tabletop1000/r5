#include "serial_device.hpp"
#include <iostream>
#include <thread>

using namespace std;

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <com port>" << std::endl;
        return EXIT_FAILURE;
    } else {

        std::cout << "Connecting to " << argv[1] << std::endl;
        std::this_thread::sleep_for(1s);
    }

    SerialDevice motor;
    motor.init(argv[1],B115200);
    float v1,v2,p1,p2;
    while(1)
    {

        motor.readData(v1,v2,p1,p2);
        cout << "V1: " << v1 << " V2: " << v2 << "P1: " << p1 << "P2: " << p2 <<endl;
    }
    return 0;
}
    


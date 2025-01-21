#include <stdio.h>
#include <thread>
#include <iostream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "zenoh.hxx"
#include <fcntl.h>
#include <csignal>
#include "BatterySensor.hpp"


using namespace zenoh;

int main(int argc, char** argv)
{

    Config config = Config::create_default();
    auto session  = Session::open(std::move(config));

    BatterySensor  jetsonBat;

    // auto pubBattery =
    //     session.declare_publisher(KeyExpr("seame/car/1/batterySensor"));
    
    jetsonBat.init("/dev/i2c-1", INA_ADDRESS, "/dev/spidev0.0");

    std::thread batteryThread(&BatterySensor::run, &jetsonBat);
    batteryThread.join();


    // while(1)
    // {
    //     usleep(10000);
    //     int buffer = this->canBus->checkReceive();
    //     if (buffer != -1) {
    //         this->canBus->readMessage(buffer);
    //     }

    //     usleep(1000); // Add a small delay to avoid busy waiting

    //     double status = this->batteryINA->readVoltage(0x02);
    //     std::cout << "Battery: " << status << std::endl;

    //     uint8_t tx[8];
    //     memcpy(tx, &status, sizeof(status));
    //     this->canBus->writeMessage(0x02, tx, sizeof(tx));
    // }

    // while (1)
    // {
    //     else if (frame.can_id == 0x02)
    //     {
    //         int battery;
    //         memcpy(&battery, frame.data, sizeof(int));
    //         battery                 = ntohl(battery);
    //         std::string battery_str = std::to_string(battery);

    //         printf("Publishing battery: '%d\n", battery);
    //         pubBattery.put(battery_str.c_str());
    //     }
    //     usleep(10);
    // }
    return 0;
}



// int main()
// {
//     try
//     {
//         BatterySensor  jetsonBat;

//         jetsonBat.init("/dev/i2c-1", INA_ADDRESS, "/dev/spidev0.0");
//         jetsonBat.run();
//     }
//     catch( std::exception &e)
//     {
//         std::cerr << e.what() << '\n';
//     }
//     return 0;
// }

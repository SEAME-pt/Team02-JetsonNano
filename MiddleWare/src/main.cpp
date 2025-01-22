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
#include "Signals.hpp"


using namespace zenoh;

int main(int argc, char** argv)
{
    BatterySensor  jetsonBat;
    Signals        allSigs;
    
    jetsonBat.init("/dev/i2c-1", INA_ADDRESS, "/dev/spidev0.0");
    allSigs.init("/dev/spidev0.0");

    std::thread batteryThread(&BatterySensor::run, &jetsonBat);
    std::thread signalsThread(&Signals::run, &allSigs);
    batteryThread.join();
    signalsThread.join();


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

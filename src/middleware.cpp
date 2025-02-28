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
    BatterySensor jetsonBat;
    Signals allSigs;

    jetsonBat.init("/dev/i2c-1", INA_ADDRESS, "/dev/spidev0.0");
    allSigs.init("/dev/spidev0.0");

    std::thread batteryThread(&BatterySensor::run, &jetsonBat);
    std::thread signalsThread(&Signals::run, &allSigs);
    batteryThread.join();
    signalsThread.join();

    return 0;
}

#include <stdio.h>
#include <thread>
#include <iostream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "zenoh.hxx"
#include <fcntl.h>
#include <csignal>
#include <memory>
#include "BatterySensor.hpp"
#include "Signals.hpp"

using namespace zenoh;

int main(int argc, char** argv)
{
    std::shared_ptr<zenoh::Session> session;
    if (argc == 2)
    {
        auto config = Config::from_file(argv[1]);
        session     = std::make_unique<zenoh::Session>(
            zenoh::Session::open(std::move(config)));
    }
    else
    {
        auto config = Config::create_default();
        session     = std::make_unique<zenoh::Session>(
            zenoh::Session::open(std::move(config)));
    }

    auto publisher = std::make_shared<SensoringPublisher>(session);
    BatterySensor jetsonBat(publisher);
    Signals allSigs(publisher);

    jetsonBat.init("/dev/i2c-1", INA_ADDRESS, "/dev/spidev0.0");
    allSigs.init("/dev/spidev0.0");

    std::thread batteryThread(&BatterySensor::run, &jetsonBat);
    std::thread signalsThread(&Signals::run, &allSigs);
    batteryThread.join();
    signalsThread.join();

    return 0;
}

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
#include "utils.hpp"

using namespace zenoh;

int main(int argc, char** argv)
{
    try
    {
        std::string configFile;
        std::string mode;

        if (!parseParameters(argc, argv, configFile, mode))
        {
            return -1;
        }

        std::shared_ptr<zenoh::Session> session;
        if (!configFile.empty())
        {
            std::cout << "Using configuration from file: " << configFile
                      << std::endl;
            auto config = Config::from_file(configFile);
            session     = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }
        else
        {
            std::cout << "Using default configuration" << std::endl;
            auto config = Config::create_default();
            // config.insert_json5("listen/endpoints",
            // "[\"udp/100.117.122.95:7450\"]");
            // config.insert_json5("connect/endpoints",
            // "[\"udp/100.117.122.95:7447\"]");
            session = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }

        auto publisher = std::make_shared<SensoringPublisher>(session);
        BatterySensor jetsonBat(publisher);
        Signals allSigs(session, publisher);

        if (mode == "local")
        {
            std::cout << "Running in LOCAL mode" << std::endl;
            jetsonBat.initLocalEnv("/dev/i2c-1", INA_ADDRESS, "can0");
            allSigs.initLocalEnv("can0");
        }
        else
        {
            std::cout << "Running in CARLA mode" << std::endl;
            jetsonBat.initCarlaEnv();
            allSigs.initCarlaEnv();
        }

        std::thread batteryThread(&BatterySensor::run, &jetsonBat);
        std::thread signalsThread(&Signals::run, &allSigs);
        batteryThread.join();
        signalsThread.join();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

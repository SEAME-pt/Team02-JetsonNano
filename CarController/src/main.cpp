#include "RaceCar.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>

int signalTo = 1;

void handler(int code)
{
    signalTo = 0;
}

int main()
{
    signal(SIGINT, handler);
    try
    {
        RaceCar raceCar;

        raceCar.init("/dev/i2c-1", 0x60, 0x40);

        std::thread runThread(&RaceCar::run, &raceCar);

        std::this_thread::sleep_for(std::chrono::seconds(1));
        {
            std::lock_guard<std::mutex> lock(raceCar.sharedData->mtx);
            raceCar.sharedData->speed     = 50;
            raceCar.sharedData->direction = 180;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        {
            std::lock_guard<std::mutex> lock(raceCar.sharedData->mtx);
            raceCar.sharedData->speed     = 70;
            raceCar.sharedData->direction = 0;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        {
            std::lock_guard<std::mutex> lock(raceCar.sharedData->mtx);
            raceCar.sharedData->speed     = 0;
            raceCar.sharedData->direction = 90;
        }

        runThread.join();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
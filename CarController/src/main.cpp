#include "RaceCar.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

int main()
{
    try
    {
        RaceCar raceCar;

        raceCar.init("/dev/i2c-1", 0x60, 0x40);

        raceCar.setSpeed(0);
        std::thread runThread(&RaceCar::run, &raceCar);

        std::this_thread::sleep_for(std::chrono::seconds(1));
        {
            std::lock_guard<std::mutex> lock(raceCar.sharedData->mtx);
            raceCar.sharedData->speed     = 50;
            raceCar.sharedData->direction = 90;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        {
            std::lock_guard<std::mutex> lock(raceCar.sharedData->mtx);
            raceCar.sharedData->speed     = 100;
            raceCar.sharedData->direction = 45;
        }

        runThread.join();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
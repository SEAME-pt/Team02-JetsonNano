#include "RaceCar.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>

int main()
{
    try
    {
        RaceCar raceCar;

        raceCar.init("/dev/i2c-1", 0x60, 0x40);
        raceCar.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
#include "RaceCar.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>

int signalTo = 1;

void handler(int code)
{
    (void)code;
    signalTo = 0;
}

int main()
{
    signal(SIGINT, handler);

    Config config = Config::create_default();
    auto session  = Session::open(std::move(config));

    try
    {
        RaceCar raceCar(session);

        raceCar.init("/dev/i2c-1", 0x60, 0x40);
        raceCar.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
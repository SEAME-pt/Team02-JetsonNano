#include "VehicleSystem.hpp"

int main(int argc, char** argv)
{
    // Start vehicle system
    VehicleSystem* system;

    if (argc == 2)
    {
        system = new VehicleSystem(argv[1]);
    }
    else
    {
        system = new VehicleSystem();
    }

    // Main loop
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    delete system;
    return 0;
}
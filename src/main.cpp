#include "VehicleSystem.hpp"

int main()
{
    // Start vehicle system
    VehicleSystem system;

    // Main loop
    while (true)
    {
        system.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
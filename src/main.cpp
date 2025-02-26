#include "VehicleFactory.hpp"

int main()
{
    Vehicle vehicle = VehicleFactory::createDefaultVehicle();

    vehicle.set_speed(50.0f);
    vehicle.set_is_moving(true);

    return 0;
}
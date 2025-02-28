#include "ChassisSteeringWheel.hpp"

int16_t ChassisSteeringWheel::get_angle() const
{
    return angle;
}

void ChassisSteeringWheel::set_angle(const int16_t value)
{
    this->angle = value;
}

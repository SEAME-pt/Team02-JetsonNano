#include "SteeringWheel.hpp"

float SteeringWheel::get_angle() const
{
    return angle;
}

void SteeringWheel::set_angle(const float value)
{
    this->angle = value;
}

float SteeringWheel::get_angle_target() const
{
    return angle_target;
}

void SteeringWheel::set_angle_target(const float value)
{
    this->angle_target = value;
}

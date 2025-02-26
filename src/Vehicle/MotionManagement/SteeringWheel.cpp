#include "SteeringWheel.hpp"

SteeringWheel::SteeringWheel() {};

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

float SteeringWheel::get_angle_target_mode() const
{
    return angle_target_mode;
}

void SteeringWheel::set_angle_target_mode(const float value)
{
    this->angle_target_mode = value;
}
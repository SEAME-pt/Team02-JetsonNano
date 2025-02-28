#include "AngularVelocity.hpp"

float AngularVelocity::get_pitch() const
{
    return pitch;
}

void AngularVelocity::set_pitch(const float value)
{
    this->pitch = value;
}

float AngularVelocity::get_roll() const
{
    return roll;
}

void AngularVelocity::set_roll(const float value)
{
    this->roll = value;
}

float AngularVelocity::get_yaw() const
{
    return yaw;
}

void AngularVelocity::set_yaw(const float value)
{
    this->yaw = value;
}

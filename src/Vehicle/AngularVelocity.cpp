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

const std::string& AngularVelocity::get_description() const
{
    return description;
}
std::string& AngularVelocity::get_mutable_description()
{
    return description;
}
void AngularVelocity::set_description(const std::string& value)
{
    this->description = value;
}

Type AngularVelocity::get_type() const
{
    return type;
}
void AngularVelocity::set_type(Type value)
{
    this->type = value;
}

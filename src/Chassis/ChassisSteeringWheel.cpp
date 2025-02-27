#include "ChassisSteeringWheel.hpp"

int16_t ChassisSteeringWheel::get_angle() const
{
    return angle;
}

void ChassisSteeringWheel::set_angle(const int16_t value)
{
    this->angle = value;
}

const std::string& ChassisSteeringWheel::get_description() const
{
    return description;
}
std::string& ChassisSteeringWheel::get_mutable_description()
{
    return description;
}
void ChassisSteeringWheel::set_description(const std::string& value)
{
    this->description = value;
}

Type ChassisSteeringWheel::get_type() const
{
    return type;
}
void ChassisSteeringWheel::set_type(Type value)
{
    this->type = value;
}

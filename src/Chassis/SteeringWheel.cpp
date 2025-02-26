#include "SteeringWheel.hpp"

SteeringWheel::SteeringWheel() {};

const int16_t SteeringWheel::get_angle() const
{
    return angle;
}
int16_t SteeringWheel::get_mutable_angle()
{
    return angle;
}
void SteeringWheel::set_angle(const int16_t value)
{
    this->angle = value;
}

const std::string& SteeringWheel::get_description() const
{
    return description;
}
std::string& SteeringWheel::get_mutable_description()
{
    return description;
}
void SteeringWheel::set_description(const std::string& value)
{
    this->description = value;
}

Type SteeringWheel::get_type() const
{
    return type;
}
void SteeringWheel::set_type(Type value)
{
    this->type = value;
}

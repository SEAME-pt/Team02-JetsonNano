#include "Steering.hpp"

Steering::Steering() {};

const SteeringWheel& Steering::get_steering_wheel() const
{
    return steering_wheel;
}
SteeringWheel& Steering::get_mutable_steering_wheel()
{
    return steering_wheel;
}
void Steering::set_steering_wheel(const SteeringWheel& value)
{
    this->steering_wheel = value;
}

const std::string& Steering::get_description() const
{
    return description;
}
std::string& Steering::get_mutable_description()
{
    return description;
}
void Steering::set_description(const std::string& value)
{
    this->description = value;
}

Type Steering::get_type() const
{
    return type;
}
void Steering::set_type(Type value)
{
    this->type = value;
}

#include "Steering.hpp"

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

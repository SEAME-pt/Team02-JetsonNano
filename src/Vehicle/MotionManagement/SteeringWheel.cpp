#include "SteeringWheel.hpp"

SteeringWheel::SteeringWheel() {};

const float SteeringWheel::get_angle() const
{
    return angle;
}
float SteeringWheel::get_mutable_angle()
{
    return angle;
}
void SteeringWheel::set_angle(const float value)
{
    this->angle = value;
}

const float SteeringWheel::get_angle_target() const
{
    return angle_target;
}
float SteeringWheel::get_mutable_angle_target()
{
    return angle_target;
}
void SteeringWheel::set_angle_target(const float value)
{
    this->angle_target = value;
}

const float SteeringWheel::get_angle_target_mode() const
{
    return angle_target_mode;
}
float SteeringWheel::get_mutable_angle_target_mode()
{
    return angle_target_mode;
}
void SteeringWheel::set_angle_target_mode(const float value)
{
    this->angle_target_mode = value;
}
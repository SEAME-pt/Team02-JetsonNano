#include "MotionManagement.hpp"

const ElectricAxle& MotionManagement::get_electric_axle() const
{
    return electric_axle;
}
ElectricAxle& MotionManagement::get_mutable_electric_axle()
{
    return electric_axle;
}
void MotionManagement::set_electric_axle(const ElectricAxle& value)
{
    this->electric_axle = value;
}

const Steering& MotionManagement::get_steering() const
{
    return steering;
}
Steering& MotionManagement::get_mutable_steering()
{
    return steering;
}
void MotionManagement::set_steering(const Steering& value)
{
    this->steering = value;
}

const std::string& MotionManagement::get_description() const
{
    return description;
}
std::string& MotionManagement::get_mutable_description()
{
    return description;
}
void MotionManagement::set_description(const std::string& value)
{
    this->description = value;
}

Type MotionManagement::get_type() const
{
    return type;
}
void MotionManagement::set_type(Type value)
{
    this->type = value;
}
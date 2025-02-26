#include "ElectricMotor.hpp"

std::uint16_t ElectricMotor::get_max_power() const
{
    return max_power;
}

void ElectricMotor::set_max_power(const std::uint16_t value)
{
    this->max_power = value;
}

std::int32_t ElectricMotor::get_speed() const
{
    return speed;
}

void ElectricMotor::set_speed(const std::int32_t value)
{
    this->speed = value;
}

float ElectricMotor::get_time_in_use() const
{
    return time_in_use;
}

void ElectricMotor::set_time_in_use(const float value)
{
    this->time_in_use = value;
}

const std::string& ElectricMotor::get_description() const
{
    return description;
}
std::string& ElectricMotor::get_mutable_description()
{
    return description;
}
void ElectricMotor::set_description(const std::string& value)
{
    this->description = value;
}

Type ElectricMotor::get_type() const
{
    return type;
}
void ElectricMotor::set_type(Type value)
{
    this->type = value;
}

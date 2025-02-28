#include "Powertrain.hpp"

const ElectricMotor& Powertrain::get_electric_motor() const
{
    return electric_motor;
}
ElectricMotor& Powertrain::get_mutable_electric_motor()
{
    return electric_motor;
}
void Powertrain::set_electric_motor(const ElectricMotor& value)
{
    this->electric_motor = value;
}

uint32_t Powertrain::get_range() const
{
    return range;
}

void Powertrain::set_range(const uint32_t value)
{
    this->range = value;
}

uint32_t Powertrain::get_time_remaining() const
{
    return time_remaining;
}

void Powertrain::set_time_remaining(const uint32_t value)
{
    this->time_remaining = value;
}

const Transmission& Powertrain::get_transmission() const
{
    return transmission;
}
Transmission& Powertrain::get_mutable_transmission()
{
    return transmission;
}
void Powertrain::set_transmission(const Transmission& value)
{
    this->transmission = value;
}

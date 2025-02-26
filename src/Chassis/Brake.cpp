#include "Brake.hpp"

Brake::Brake() {};

const bool Brake::get_is_driver_emergency_braking_detected() const
{
    return is_driver_emergency_braking_detected;
}
bool Brake::get_mutable_is_driver_emergency_braking_detected()
{
    return is_driver_emergency_braking_detected;
}
void Brake::set_is_driver_emergency_braking_detected(const bool value)
{
    this->is_driver_emergency_braking_detected = value;
}

const std::uint8_t Brake::get_pedal_position() const
{
    return pedal_position;
}
std::uint8_t Brake::get_mutable_pedal_position()
{
    return pedal_position;
}
void Brake::set_pedal_position(const std::uint8_t value)
{
    this->pedal_position = value;
}

const std::string& Brake::get_description() const
{
    return description;
}
std::string& Brake::get_mutable_description()
{
    return description;
}
void Brake::set_description(const std::string& value)
{
    this->description = value;
}

Type Brake::get_type() const
{
    return type;
}
void Brake::set_type(Type value)
{
    this->type = value;
}

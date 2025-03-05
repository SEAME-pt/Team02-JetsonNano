#include "Brake.hpp"

bool Brake::get_is_driver_emergency_braking_detected() const
{
    return is_driver_emergency_braking_detected;
}

void Brake::set_is_driver_emergency_braking_detected(const bool value)
{
    this->is_driver_emergency_braking_detected = value;
}

std::uint8_t Brake::get_pedal_position() const
{
    return pedal_position;
}

void Brake::set_pedal_position(const std::uint8_t value)
{
    this->pedal_position = value;
}

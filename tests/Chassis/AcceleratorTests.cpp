#include "Accelerator.hpp"

std::uint8_t Accelerator::get_pedal_position() const
{
    return pedal_position;
}

void Accelerator::set_pedal_position(const std::uint8_t value)
{
    this->pedal_position = value;
}

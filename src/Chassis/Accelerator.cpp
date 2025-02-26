#include "Accelerator.hpp"

Accelerator::Accelerator() {};

const std::uint8_t Accelerator::get_pedal_position() const
{
    return pedal_position;
}
std::uint8_t Accelerator::get_mutable_pedal_position()
{
    return pedal_position;
}
void Accelerator::set_pedal_position(const std::uint8_t value)
{
    this->pedal_position = value;
}

const std::string& Accelerator::get_description() const
{
    return description;
}
std::string& Accelerator::get_mutable_description()
{
    return description;
}
void Accelerator::set_description(const std::string& value)
{
    this->description = value;
}

Type Accelerator::get_type() const
{
    return type;
}
void Accelerator::set_type(Type value)
{
    this->type = value;
}
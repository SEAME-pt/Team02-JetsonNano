#include "Acceleration.hpp"

float Acceleration::get_lateral() const
{
    return lateral;
}

void Acceleration::set_lateral(const float value)
{
    this->lateral = value;
}

float Acceleration::get_longitudinal() const
{
    return longitudinal;
}

void Acceleration::set_longitudinal(const float value)
{
    this->longitudinal = value;
}

float Acceleration::get_vertical() const
{
    return vertical;
}

void Acceleration::set_vertical(const float value)
{
    this->vertical = value;
}

const std::string& Acceleration::get_description() const
{
    return description;
}
std::string& Acceleration::get_mutable_description()
{
    return description;
}
void Acceleration::set_description(const std::string& value)
{
    this->description = value;
}

Type Acceleration::get_type() const
{
    return type;
}
void Acceleration::set_type(Type value)
{
    this->type = value;
}

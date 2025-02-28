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

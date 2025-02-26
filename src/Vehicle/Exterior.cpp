#include "Exterior.hpp"

Exterior::Exterior() {};

const float Exterior::get_air_temperature() const
{
    return air_temperature;
}
void Exterior::set_air_temperature(const float value)
{
    this->air_temperature = value;
}

const float Exterior::get_humidity() const
{
    return humidity;
}
void Exterior::set_humidity(const float value)
{
    this->humidity = value;
}

const float Exterior::get_light_intensity() const
{
    return light_intensity;
}
void Exterior::set_light_intensity(const float value)
{
    this->light_intensity = value;
}

const std::string& Exterior::get_description() const
{
    return description;
}
std::string& Exterior::get_mutable_description()
{
    return description;
}
void Exterior::set_description(const std::string& value)
{
    this->description = value;
}

Type Exterior::get_type() const
{
    return type;
}
void Exterior::set_type(Type value)
{
    this->type = value;
}

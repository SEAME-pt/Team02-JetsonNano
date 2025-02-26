#include "Transmission.hpp"

Transmission::Transmission() {};

std::int8_t Transmission::get_current_gear() const
{
    return current_gear;
}

void Transmission::set_current_gear(const std::int8_t value)
{
    this->current_gear = value;
}

const std::string Transmission::get_drive_type() const
{
    return drive_type;
}
std::string Transmission::get_mutable_drive_type()
{
    return drive_type;
}
void Transmission::set_drive_type(const std::string value)
{
    this->drive_type = value;
}

const std::string& Transmission::get_gear_change_mode() const
{
    return gear_change_mode;
}
std::string& Transmission::get_mutable_gear_change_mode()
{
    return gear_change_mode;
}
void Transmission::set_gear_change_mode(const std::string& value)
{
    this->gear_change_mode = value;
}

bool Transmission::get_is_park_lock_engaged() const
{
    return is_park_lock_engaged;
}

void Transmission::set_is_park_lock_engaged(const bool value)
{
    this->is_park_lock_engaged = value;
}

const std::string& Transmission::get_performance_mode() const
{
    return performance_mode;
}
std::string& Transmission::get_mutable_performance_mode()
{
    return performance_mode;
}
void Transmission::set_performance_mode(const std::string& value)
{
    this->performance_mode = value;
}

std::int8_t Transmission::get_selected_gear() const
{
    return selected_gear;
}

void Transmission::set_selected_gear(const std::int8_t value)
{
    this->selected_gear = value;
}

float Transmission::get_travelled_distance() const
{
    return travelled_distance;
}

void Transmission::set_travelled_distance(const float value)
{
    this->travelled_distance = value;
}

const std::string& Transmission::get_description() const
{
    return description;
}
std::string& Transmission::get_mutable_description()
{
    return description;
}
void Transmission::set_description(const std::string& value)
{
    this->description = value;
}

Type Transmission::get_type() const
{
    return type;
}
void Transmission::set_type(Type value)
{
    this->type = value;
}

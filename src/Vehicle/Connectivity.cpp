#include "Connectivity.hpp"

Connectivity::Connectivity() {};

bool Connectivity::get_is_connectivity_available() const
{
    return is_connectivity_available;
}

void Connectivity::set_is_connectivity_available(const bool value)
{
    this->is_connectivity_available = value;
}

const std::string& Connectivity::get_description() const
{
    return description;
}
std::string& Connectivity::get_mutable_description()
{
    return description;
}
void Connectivity::set_description(const std::string& value)
{
    this->description = value;
}

Type Connectivity::get_type() const
{
    return type;
}
void Connectivity::set_type(Type value)
{
    this->type = value;
}

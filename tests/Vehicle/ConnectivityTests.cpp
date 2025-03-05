#include "Connectivity.hpp"

bool Connectivity::get_is_connectivity_available() const
{
    return is_connectivity_available;
}

void Connectivity::set_is_connectivity_available(const bool value)
{
    this->is_connectivity_available = value;
}

#include "Body.hpp"

const Lights& Body::get_lights() const
{
    return lights;
}
Lights& Body::get_mutable_lights()
{
    return lights;
}
void Body::set_lights(const Lights& value)
{
    this->lights = value;
}

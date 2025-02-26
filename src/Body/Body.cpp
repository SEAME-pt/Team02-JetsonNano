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

const std::string& Body::get_description() const
{
    return description;
}
std::string& Body::get_mutable_description()
{
    return description;
}
void Body::set_description(const std::string& value)
{
    this->description = value;
}

Type Body::get_type() const
{
    return type;
}
void Body::set_type(Type value)
{
    this->type = value;
}
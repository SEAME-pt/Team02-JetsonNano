#include "Lights.hpp"

bool SignalingLights::get_is_defect() const
{
    return is_defect;
}

void SignalingLights::set_is_defect(const bool value)
{
    this->is_defect = value;
}

bool SignalingLights::get_is_signaling() const
{
    return is_signaling;
}

void SignalingLights::set_is_signaling(const bool value)
{
    this->is_signaling = value;
}

bool StaticLights::get_is_defect() const
{
    return is_defect;
}

void StaticLights::set_is_defect(const bool value)
{
    this->is_defect = value;
}

bool StaticLights::get_is_on() const
{
    return is_on;
}

void StaticLights::set_is_on(const bool value)
{
    this->is_on = value;
}

bool BrakeLights::get_is_defect() const
{
    return is_defect;
}

void BrakeLights::set_is_defect(const bool value)
{
    this->is_defect = value;
}

bool BrakeLights::get_is_active() const
{
    return is_active;
}

void BrakeLights::set_is_active(const bool value)
{
    this->is_active = value;
}

const StaticLights& Lights::get_beam() const
{
    return beam;
}
StaticLights& Lights::get_mutable_beam()
{
    return beam;
}
void Lights::set_beam(const StaticLights& value)
{
    this->beam = value;
}

const BrakeLights& Lights::get_brake() const
{
    return brake;
}
BrakeLights& Lights::get_mutable_brake()
{
    return brake;
}
void Lights::set_brake(const BrakeLights& value)
{
    this->brake = value;
}

const SignalingLights& Lights::get_direction_indicator() const
{
    return direction_indicator;
}
SignalingLights& Lights::get_mutable_direction_indicator()
{
    return direction_indicator;
}
void Lights::set_direction_indicator(const SignalingLights& value)
{
    this->direction_indicator = value;
}

const StaticLights& Lights::get_fog() const
{
    return fog;
}
StaticLights& Lights::get_mutable_fog()
{
    return fog;
}
void Lights::set_fog(const StaticLights& value)
{
    this->fog = value;
}

const SignalingLights& Lights::get_hazard() const
{
    return hazard;
}
SignalingLights& Lights::get_mutable_hazard()
{
    return hazard;
}
void Lights::set_hazard(const SignalingLights& value)
{
    this->hazard = value;
}

const std::string& Lights::get_light_switch() const
{
    return light_switch;
}
std::string& Lights::get_mutable_light_switch()
{
    return light_switch;
}
void Lights::set_light_switch(const std::string& value)
{
    this->light_switch = value;
}

const StaticLights& Lights::get_parking() const
{
    return parking;
}
StaticLights& Lights::get_mutable_parking()
{
    return parking;
}
void Lights::set_parking(const StaticLights& value)
{
    this->parking = value;
}

const StaticLights& Lights::get_running() const
{
    return running;
}
StaticLights& Lights::get_mutable_running()
{
    return running;
}
void Lights::set_running(const StaticLights& value)
{
    this->running = value;
}

const std::string& Lights::get_description() const
{
    return description;
}
std::string& Lights::get_mutable_description()
{
    return description;
}
void Lights::set_description(const std::string& value)
{
    this->description = value;
}

Type Lights::get_type() const
{
    return type;
}
void Lights::set_type(Type value)
{
    this->type = value;
}

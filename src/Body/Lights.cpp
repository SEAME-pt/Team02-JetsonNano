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

const StaticLights& Lights::get_beam_low() const
{
    return beam_low;
}
StaticLights& Lights::get_mutable_beam_low()
{
    return beam_low;
}
void Lights::set_beam_low(const StaticLights& value)
{
    this->beam_low = value;
}

const StaticLights& Lights::get_beam_high() const
{
    return beam_high;
}
StaticLights& Lights::get_mutable_beam_high()
{
    return beam_high;
}
void Lights::set_beam_high(const StaticLights& value)
{
    this->beam_high = value;
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

const SignalingLights& Lights::get_direction_indicator_left() const
{
    return direction_indicator_left;
}
SignalingLights& Lights::get_mutable_direction_indicator_left()
{
    return direction_indicator_left;
}
void Lights::set_direction_indicator_left(const SignalingLights& value)
{
    this->direction_indicator_left = value;
}

const SignalingLights& Lights::get_direction_indicator_right() const
{
    return direction_indicator_right;
}
SignalingLights& Lights::get_mutable_direction_indicator_right()
{
    return direction_indicator_right;
}
void Lights::set_direction_indicator_right(const SignalingLights& value)
{
    this->direction_indicator_right = value;
}

const StaticLights& Lights::get_fog_rear() const
{
    return fog_rear;
}
StaticLights& Lights::get_mutable_fog_rear()
{
    return fog_rear;
}
void Lights::set_fog_rear(const StaticLights& value)
{
    this->fog_rear = value;
}

const StaticLights& Lights::get_fog_front() const
{
    return fog_front;
}
StaticLights& Lights::get_mutable_fog_front()
{
    return fog_front;
}
void Lights::set_fog_front(const StaticLights& value)
{
    this->fog_front = value;
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

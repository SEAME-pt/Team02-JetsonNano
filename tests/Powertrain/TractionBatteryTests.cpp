#include "TractionBattery.hpp"

float TractionBattery::get_state_of_charge_displayed() const
{
    return state_of_charge_displayed;
}

float TractionBattery::get_mutable_state_of_charge_displayed()
{
    return state_of_charge_displayed;
}   

void TractionBattery::set_state_of_charge_displayed(const float value)
{
    this->state_of_charge_displayed = value;
}   

std::uint16_t TractionBattery::get_max_voltage() const
{
    return max_voltage;
}

void TractionBattery::set_max_voltage(const std::uint16_t value)
{
    this->max_voltage = value;
}

float TractionBattery::get_current_voltage() const
{
    return current_voltage;
}

float TractionBattery::get_mutable_current_voltage()
{
    return current_voltage;
}

void TractionBattery::set_current_voltage(const float value)
{
    this->current_voltage = value;
}

float TractionBattery::get_current_current() const
{
    return current_current;
}

float TractionBattery::get_mutable_current_current()
{
    return current_current;
}

void TractionBattery::set_current_current(const float value)
{
    this->current_current = value;
}

float TractionBattery::get_current_power() const
{
    return current_power;
}

float TractionBattery::get_mutable_current_power()
{
    return current_power;
}

void TractionBattery::set_current_power(const float value)
{
    this->current_power = value;
}


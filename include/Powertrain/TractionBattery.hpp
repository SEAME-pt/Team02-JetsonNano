#pragma once

#include <iostream>

/**
 * @brief High voltage traction battery management system
 *
 * @details Monitors and controls the high voltage battery including:
 *          - State of charge monitoring
 *          - Voltage management
 *          - Current monitoring
 *          - Power calculations
 *
 * Key measurements:
 *  - State of charge (0-100%)
 *  - Current (A, positive=charging, negative=discharging)
 *  - Power (W, positive=charging, negative=discharging)
 *  - Voltage (V)
 *
 * @note Implements VSS Powertrain.TractionBattery branch specification
 * @note All electrical measurements follow standard SI units
 */
class TractionBattery
{
  public:
    TractionBattery()          = default;
    virtual ~TractionBattery() = default;

  private:
    float state_of_charge_displayed;
    std::uint16_t max_voltage;
    float current_voltage;
    float current_current;
    float current_power;

  public:
    float get_state_of_charge_displayed() const;
    float get_mutable_state_of_charge_displayed();
    void set_state_of_charge_displayed(const float value);

    std::uint16_t get_max_voltage() const;
    void set_max_voltage(const std::uint16_t value);

    float get_current_voltage() const;
    float get_mutable_current_voltage();
    void set_current_voltage(const float value);

    float get_current_current() const;
    float get_mutable_current_current();
    void set_current_current(const float value);

    float get_current_power() const;
    float get_mutable_current_power();
    void set_current_power(const float value);
};

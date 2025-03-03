#pragma once

#include "ElectricMotor.hpp"
#include "Transmission.hpp"
#include "TractionBattery.hpp"
#include <iostream>

class Powertrain
{
  public:
    Powertrain()          = default;
    virtual ~Powertrain() = default;

  private:
    Transmission transmission;
    ElectricMotor electric_motor;
    TractionBattery traction_battery;
    std::uint32_t range;
    std::uint32_t time_remaining;

  public:
    const ElectricMotor& get_electric_motor() const;
    ElectricMotor& get_mutable_electric_motor();
    void set_electric_motor(const ElectricMotor& value);

    const TractionBattery& get_traction_battery() const;
    TractionBattery& get_mutable_traction_battery();
    void set_traction_battery(const TractionBattery& value);

    uint32_t get_range() const;
    void set_range(const uint32_t value);

    uint32_t get_time_remaining() const;
    void set_time_remaining(const uint32_t value);

    const Transmission& get_transmission() const;
    Transmission& get_mutable_transmission();
    void set_transmission(const Transmission& value);
};
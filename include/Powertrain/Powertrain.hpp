#pragma once

#include "ElectricMotor.hpp"
#include "Transmission.hpp"
#include "Type.hpp"
#include <iostream>

class Powertrain
{
  public:
    Powertrain()          = default;
    virtual ~Powertrain() = default;

  private:
    Transmission transmission;
    ElectricMotor electric_motor;
    std::uint32_t range;
    std::uint32_t time_remaining;
    std::string description;
    Type type;

  public:
    const ElectricMotor& get_electric_motor() const;
    ElectricMotor& get_mutable_electric_motor();
    void set_electric_motor(const ElectricMotor& value);

    const uint32_t get_range() const;
    uint32_t get_mutable_range();
    void set_range(const uint32_t value);

    const uint32_t get_time_remaining() const;
    uint32_t get_mutable_time_remaining();
    void set_time_remaining(const uint32_t value);

    const Transmission& get_transmission() const;
    Transmission& get_mutable_transmission();
    void set_transmission(const Transmission& value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};
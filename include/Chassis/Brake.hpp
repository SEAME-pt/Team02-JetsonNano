#pragma once

#include "Type.hpp"
#include <iostream>

class Brake
{
  public:
    Brake()          = default;
    virtual ~Brake() = default;

  private:
    bool is_driver_emergency_braking_detected;
    std::uint8_t pedal_position;
    std::string description;
    Type type;

  public:
    bool get_is_driver_emergency_braking_detected() const;
    void set_is_driver_emergency_braking_detected(const bool value);

    std::uint8_t get_pedal_position() const;
    void set_pedal_position(const std::uint8_t value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};
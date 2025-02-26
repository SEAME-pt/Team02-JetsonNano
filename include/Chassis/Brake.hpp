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
    const bool get_is_driver_emergency_braking_detected() const;
    bool get_mutable_is_driver_emergency_braking_detected();
    void set_is_driver_emergency_braking_detected(const bool value);

    const std::uint8_t get_pedal_position() const;
    std::uint8_t get_mutable_pedal_position();
    void set_pedal_position(const std::uint8_t value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);
    Type get_type() const;
    void set_type(Type value);
};
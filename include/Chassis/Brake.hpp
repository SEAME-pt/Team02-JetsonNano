#pragma once

#include <iostream>

class Brake
{
  public:
    Brake()          = default;
    virtual ~Brake() = default;

  private:
    bool is_driver_emergency_braking_detected;
    std::uint8_t pedal_position;

  public:
    bool get_is_driver_emergency_braking_detected() const;
    void set_is_driver_emergency_braking_detected(const bool value);

    std::uint8_t get_pedal_position() const;
    void set_pedal_position(const std::uint8_t value);
};
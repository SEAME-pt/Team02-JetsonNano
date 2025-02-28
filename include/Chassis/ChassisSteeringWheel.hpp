#pragma once

#include <iostream>

class ChassisSteeringWheel
{
  public:
    ChassisSteeringWheel()          = default;
    virtual ~ChassisSteeringWheel() = default;

  private:
    std::int16_t angle;

  public:
    int16_t get_angle() const;
    void set_angle(const int16_t value);
};

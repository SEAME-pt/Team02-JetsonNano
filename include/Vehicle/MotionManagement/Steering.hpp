#pragma once

#include "SteeringWheel.hpp"
#include <iostream>

class Steering
{
  public:
    Steering()          = default;
    virtual ~Steering() = default;

  private:
    SteeringWheel steering_wheel;

  public:
    const SteeringWheel& get_steering_wheel() const;
    SteeringWheel& get_mutable_steering_wheel();
    void set_steering_wheel(const SteeringWheel& value);
};

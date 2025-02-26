#pragma once

#include "Type.hpp"
#include <iostream>

class SteeringWheel
{
  public:
    SteeringWheel()          = default;
    virtual ~SteeringWheel() = default;

  private:
    float angle;
    float angle_target;
    float angle_target_mode;

  public:
    float get_angle() const;
    void set_angle(const float value);

    float get_angle_target() const;
    void set_angle_target(const float value);

    float get_angle_target_mode() const;
    void set_angle_target_mode(const float value);
};
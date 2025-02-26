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
    const float get_angle() const;
    float get_mutable_angle();
    void set_angle(const float value);

    const float get_angle_target() const;
    float get_mutable_angle_target();
    void set_angle_target(const float value);

    const float get_angle_target_mode() const;
    float get_mutable_angle_target_mode();
    void set_angle_target_mode(const float value);
};
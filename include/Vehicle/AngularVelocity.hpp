#pragma once

#include <iostream>

class AngularVelocity
{
  public:
    AngularVelocity()          = default;
    virtual ~AngularVelocity() = default;

  private:
    float pitch;
    float roll;
    float yaw;

  public:
    float get_pitch() const;
    void set_pitch(const float value);

    float get_roll() const;
    void set_roll(const float value);

    float get_yaw() const;
    void set_yaw(const float value);
};

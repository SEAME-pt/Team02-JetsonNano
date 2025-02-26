#pragma once

#include "Type.hpp"
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
    std::string description;
    Type type;

  public:
    const float get_pitch() const;
    float get_mutable_pitch();
    void set_pitch(const float value);

    const float get_roll() const;
    float get_mutable_roll();
    void set_roll(const float value);

    const float get_yaw() const;
    float get_mutable_yaw();
    void set_yaw(const float value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

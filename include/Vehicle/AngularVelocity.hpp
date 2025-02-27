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
    float get_pitch() const;
    void set_pitch(const float value);

    float get_roll() const;
    void set_roll(const float value);

    float get_yaw() const;
    void set_yaw(const float value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

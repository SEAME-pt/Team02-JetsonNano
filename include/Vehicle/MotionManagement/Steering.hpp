#pragma once

#include "Type.hpp"
#include "SteeringWheel.hpp"
#include <iostream>

class Steering
{
  public:
    Steering()          = default;
    virtual ~Steering() = default;

  private:
    SteeringWheel steering_wheel;
    std::string description;
    Type type;

  public:
    const SteeringWheel& get_steering_wheel() const;
    SteeringWheel& get_mutable_steering_wheel();
    void set_steering_wheel(const SteeringWheel& value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

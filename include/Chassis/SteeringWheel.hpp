#pragma once

#include "Type.hpp"
#include <iostream>

class ChassisChassisSteeringWheel
{
  public:
    ChassisSteeringWheel()          = default;
    virtual ~ChassisSteeringWheel() = default;

  private:
    std::int16_t angle;
    std::string description;
    Type type;

  public:
    int16_t get_angle() const;
    void set_angle(const int16_t value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

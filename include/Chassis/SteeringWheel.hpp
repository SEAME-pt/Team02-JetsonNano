#pragma once

#include "Type.hpp"
#include <iostream>

class SteeringWheel
{
  public:
    SteeringWheel()          = default;
    virtual ~SteeringWheel() = default;

  private:
    std::int16_t angle;
    std::string description;
    Type type;

  public:
    const int16_t get_angle() const;
    int16_t get_mutable_angle();
    void set_angle(const int16_t value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

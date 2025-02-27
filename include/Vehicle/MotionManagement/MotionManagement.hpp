#pragma once

#include "Type.hpp"
#include "Steering.hpp"
#include "ElectricAxle.hpp"
#include <iostream>

class MotionManagement
{
  public:
    MotionManagement()          = default;
    virtual ~MotionManagement() = default;

  private:
    ElectricAxle electric_axle;
    Steering steering;
    std::string description;
    Type type;

  public:
    const ElectricAxle& get_electric_axle() const;
    ElectricAxle& get_mutable_electric_axle();
    void set_electric_axle(const ElectricAxle& value);

    const Steering& get_steering() const;
    Steering& get_mutable_steering();
    void set_steering(const Steering& value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};
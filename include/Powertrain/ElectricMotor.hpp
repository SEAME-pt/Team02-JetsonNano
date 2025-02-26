#pragma once

#include "Type.hpp"
#include <iostream>

class ElectricMotor
{
  public:
    ElectricMotor()          = default;
    virtual ~ElectricMotor() = default;

  private:
    std::uint16_t max_power;
    std::int32_t speed;
    float time_in_use;
    std::string description;
    Type type;

  public:
    const std::uint16_t get_max_power() const;
    std::uint16_t get_mutable_max_power();
    void set_max_power(const std::uint16_t value);

    const std::int32_t get_speed() const;
    std::int32_t get_mutable_speed();
    void set_speed(const std::int32_t value);

    const float get_time_in_use() const;
    float get_mutable_time_in_use();
    void set_time_in_use(const float value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

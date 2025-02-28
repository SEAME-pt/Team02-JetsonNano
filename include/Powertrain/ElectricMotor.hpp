#pragma once

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

  public:
    std::uint16_t get_max_power() const;
    void set_max_power(const std::uint16_t value);

    std::int32_t get_speed() const;
    void set_speed(const std::int32_t value);

    float get_time_in_use() const;
    void set_time_in_use(const float value);
};

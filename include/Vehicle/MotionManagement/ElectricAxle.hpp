#pragma once

#include <iostream>

class ElectricAxleRow
{
  public:
    ElectricAxleRow()          = default;
    virtual ~ElectricAxleRow() = default;

  private:
    std::uint16_t rotational_speed;
    std::uint16_t rotational_speed_target;

  public:
    std::uint16_t get_rotational_speed() const;
    void set_rotational_speed(const std::uint16_t value);

    std::uint16_t get_rotational_speed_target() const;
    void set_rotational_speed_target(const std::uint16_t value);
};

class ElectricAxle
{
  public:
    ElectricAxle()          = default;
    virtual ~ElectricAxle() = default;

  private:
    ElectricAxleRow row1;
    ElectricAxleRow row2;

  public:
    const ElectricAxleRow& get_row1() const;
    ElectricAxleRow& get_mutable_row1();
    void set_row1(const ElectricAxleRow& value);

    const ElectricAxleRow& get_row2() const;
    ElectricAxleRow& get_mutable_row2();
    void set_row2(const ElectricAxleRow& value);
};
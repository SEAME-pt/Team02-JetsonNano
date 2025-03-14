#pragma once

#include <iostream>

/**
 * @brief Single electric axle row control
 *
 * @details Manages individual axle row characteristics:
 *          - Current rotational speed
 *          - Target rotational speed
 *          - Direction control
 *
 * @note All speeds in RPM
 */
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

/**
 * @brief Electric axle row control system
 *
 * @details Manages electric axle functionality for up to 2 rows:
 *          - Speed control
 *          - Direction management
 *          - Torque distribution
 *          - Regenerative braking
 *
 * Supports:
 * - Row 1 (Front axle)
 * - Row 2 (Rear axle)
 *
 * @note All speeds in RPM
 * @note Implements VSS Vehicle.MotionManagement.ElectricAxle branch
 * specification
 */
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
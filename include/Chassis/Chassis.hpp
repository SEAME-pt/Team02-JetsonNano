#pragma once

#include "Axle.hpp"
#include "Brake.hpp"
#include "Accelerator.hpp"
#include "ChassisSteeringWheel.hpp"
#include <iostream>

/**
 * @brief Vehicle chassis management class
 *
 * @details Controls and monitors all chassis-related systems including:
 *          - Axle configuration and management
 *          - Steering control
 *          - Brake system
 *          - Accelerator control
 *          - Wheel management
 *
 * @note Implements VSS Chassis branch specification
 * @see Axle
 * @see ChassisSteeringWheel
 * @see Brake
 * @see Accelerator
 */
class Chassis
{
  public:
    Chassis()          = default;
    virtual ~Chassis() = default;

  private:
    std::uint16_t wheelbase;
    Axle axle;
    std::uint8_t axle_count;
    ChassisSteeringWheel steering_wheel;
    Accelerator accelerator;
    Brake brake;

  public:
    const Accelerator& get_accelerator() const;
    Accelerator& get_mutable_accelerator();
    void set_accelerator(const Accelerator& value);

    const Axle& get_axle() const;
    Axle& get_mutable_axle();
    void set_axle(const Axle& value);

    std::uint8_t get_axle_count() const;
    void set_axle_count(const std::uint8_t value);

    const Brake& get_brake() const;
    Brake& get_mutable_brake();
    void set_brake(const Brake& value);

    const ChassisSteeringWheel& get_steering_wheel() const;
    ChassisSteeringWheel& get_mutable_steering_wheel();
    void set_steering_wheel(const ChassisSteeringWheel& value);

    std::uint16_t get_wheelbase() const;
    void set_wheelbase(const std::uint16_t value);
};

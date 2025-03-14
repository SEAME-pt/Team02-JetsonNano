#pragma once

#include "Steering.hpp"
#include "ElectricAxle.hpp"
#include <iostream>

/**
 * @brief Vehicle motion management system
 *
 * @details Controls overall vehicle motion including:
 *          - Electric axle control
 *          - Steering management
 *          - Motion coordination
 *          - Acceleration and deceleration control
 *
 * Manages the integration between:
 * - Electric axle systems (up to 2 rows)
 * - Steering systems
 * - Motion control algorithms
 *
 * @note Implements VSS Vehicle.MotionManagement branch specification
 * @see ElectricAxle
 * @see Steering
 */
class MotionManagement
{
  public:
    MotionManagement()          = default;
    virtual ~MotionManagement() = default;

  private:
    ElectricAxle electric_axle;
    Steering steering;

  public:
    const ElectricAxle& get_electric_axle() const;
    ElectricAxle& get_mutable_electric_axle();
    void set_electric_axle(const ElectricAxle& value);

    const Steering& get_steering() const;
    Steering& get_mutable_steering();
    void set_steering(const Steering& value);
};
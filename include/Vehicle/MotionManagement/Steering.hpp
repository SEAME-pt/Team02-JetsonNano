#pragma once

#include "SteeringWheel.hpp"
#include <iostream>

/**
 * @brief Vehicle steering management system
 *
 * @details Coordinates steering functionality including:
 *          - Steering wheel position control
 *          - Multi-axle steering coordination
 *          - Steering assistance management
 *
 * Features:
 * - Steering wheel angle monitoring
 * - Multiple steering rack support
 * - Steering assistance levels
 *
 * @note Implements VSS Vehicle.MotionManagement.Steering branch specification
 * @see SteeringWheel
 */
class Steering
{
  public:
    Steering()          = default;
    virtual ~Steering() = default;

  private:
    SteeringWheel steering_wheel;

  public:
    const SteeringWheel& get_steering_wheel() const;
    SteeringWheel& get_mutable_steering_wheel();
    void set_steering_wheel(const SteeringWheel& value);
};

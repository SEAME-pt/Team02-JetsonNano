#pragma once

#include <iostream>

/**
 * @brief Steering wheel control system
 *
 * @details Manages steering wheel position and control:
 *          - Current angle (-180 to +180 degrees)
 *          - Target angle
 *          - ISO 8855 compliant measurements
 *
 * @note Positive angle = counterclockwise (ISO 8855)
 * @note Implements VSS Vehicle.MotionManagement.Steering.SteeringWheel
 * specification
 */
class SteeringWheel
{
  public:
    SteeringWheel()          = default;
    virtual ~SteeringWheel() = default;

  private:
    float angle;
    float angle_target;

  public:
    float get_angle() const;
    void set_angle(const float value);

    float get_angle_target() const;
    void set_angle_target(const float value);
};
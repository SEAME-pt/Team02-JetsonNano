#pragma once

#include <iostream>

/**
 * @brief Vehicle angular velocity measurement system
 *
 * @details Monitors vehicle rotation rates:
 *          - Roll (X axis)
 *          - Pitch (Y axis)
 *          - Yaw (Z axis)
 *
 * @note All measurements in degrees/second
 * @note Axis definitions according to ISO 8855
 * @note Implements VSS Vehicle.AngularVelocity branch specification
 */
class AngularVelocity
{
  public:
    AngularVelocity()          = default;
    virtual ~AngularVelocity() = default;

  private:
    float pitch;
    float roll;
    float yaw;

  public:
    float get_pitch() const;
    void set_pitch(const float value);

    float get_roll() const;
    void set_roll(const float value);

    float get_yaw() const;
    void set_yaw(const float value);
};

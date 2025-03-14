#pragma once

#include <iostream>

/**
 * @brief Vehicle acceleration measurement system
 *
 * @details Monitors vehicle acceleration in three axes:
 *          - Longitudinal (X axis)
 *          - Lateral (Y axis)
 *          - Vertical (Z axis)
 *
 * @note All measurements in m/s²
 * @note Axis definitions according to ISO 8855
 * @note Implements VSS Vehicle.Acceleration branch specification
 */
class Acceleration
{
  public:
    Acceleration()          = default;
    virtual ~Acceleration() = default;

  private:
    float lateral;
    float longitudinal;
    float vertical;

  public:
    float get_lateral() const;
    void set_lateral(const float value);

    float get_longitudinal() const;
    void set_longitudinal(const float value);

    float get_vertical() const;
    void set_vertical(const float value);
};
#pragma once

#include <iostream>

/**
 * @brief Vehicle exterior environment monitoring system
 *
 * @details Monitors external environmental conditions:
 *          - Air temperature (Celsius)
 *          - Humidity (0-100%)
 *          - Light intensity (0-100%)
 *
 * @note Temperature in Celsius
 * @note Humidity and light intensity as percentages
 * @note Implements VSS Vehicle.Exterior branch specification
 */
class Exterior
{
  public:
    Exterior()          = default;
    virtual ~Exterior() = default;

  private:
    float air_temperature;
    float humidity;
    float light_intensity;

  public:
    float get_air_temperature() const;
    void set_air_temperature(const float value);

    float get_humidity() const;
    void set_humidity(const float value);

    float get_light_intensity() const;
    void set_light_intensity(const float value);
};
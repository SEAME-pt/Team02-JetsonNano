#pragma once

#include <iostream>

/**
 * @brief Vehicle transmission system management
 *
 * @details Controls transmission functionality including:
 *          - Gear selection and monitoring
 *          - Drive type configuration
 *          - Performance modes
 *          - Park lock management
 *
 * Supported drive types:
 *  - FORWARD_WHEEL_DRIVE
 *  - REAR_WHEEL_DRIVE
 *  - ALL_WHEEL_DRIVE
 *
 * Performance modes:
 *  - NORMAL
 *  - SPORT
 *  - ECONOMY
 *  - SNOW
 *  - RAIN
 *
 * @note Gear values: 0=Neutral, +N=Forward, -N=Reverse, 126=Park, 127=Drive
 * @note Implements VSS Powertrain.Transmission branch specification
 */
class Transmission
{
  public:
    Transmission()          = default;
    virtual ~Transmission() = default;

  private:
    std::int8_t current_gear;
    std::string drive_type;
    std::string gear_change_mode;
    bool is_park_lock_engaged;
    std::string performance_mode;
    std::int8_t selected_gear;
    float travelled_distance;

  public:
    std::int8_t get_current_gear() const;
    void set_current_gear(const std::int8_t value);

    const std::string& get_drive_type() const;
    std::string& get_mutable_drive_type();
    void set_drive_type(const std::string value);

    const std::string& get_gear_change_mode() const;
    std::string& get_mutable_gear_change_mode();
    void set_gear_change_mode(const std::string& value);

    bool get_is_park_lock_engaged() const;
    void set_is_park_lock_engaged(const bool value);

    const std::string& get_performance_mode() const;
    std::string& get_mutable_performance_mode();
    void set_performance_mode(const std::string& value);

    std::int8_t get_selected_gear() const;
    void set_selected_gear(const std::int8_t value);

    float get_travelled_distance() const;
    void set_travelled_distance(const float value);
};
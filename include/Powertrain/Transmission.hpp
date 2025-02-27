#pragma once

#include "Type.hpp"
#include <iostream>

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
    std::string description;
    Type type;

  public:
    std::int8_t get_current_gear() const;
    void set_current_gear(const std::int8_t value);

    const std::string get_drive_type() const;
    std::string get_mutable_drive_type();
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

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};
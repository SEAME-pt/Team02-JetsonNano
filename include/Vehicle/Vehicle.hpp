#pragma once

#include "Powertrain.hpp"
#include "Body.hpp"
#include "ADAS.hpp"
#include "Chassis.hpp"
#include "Exterior.hpp"
#include "Connectivity.hpp"
#include "Acceleration.hpp"
#include "AngularVelocity.hpp"
#include "MotionManagement.hpp"

using Default = std::variant<int64_t, std::string>;

class Vehicle
{
  public:
    Vehicle()          = default;
    virtual ~Vehicle() = default;

  private:
    Powertrain powertrain;
    Body body;
    ADAS adas;
    Chassis chassis;
    Exterior exterior;
    Connectivity connectivity;
    float speed;
    float traveled_distance;
    float traveled_distance_since_start;
    std::string start_time;
    float trip_duration;
    bool is_moving;
    float average_speed;
    Acceleration acceleration;
    AngularVelocity angular_velocity;
    std::uint16_t current_overall_weight;
    std::uint16_t length;
    std::uint16_t height;
    std::uint16_t turning_diameter;
    MotionManagement motion_management;
    std::string description;
    Type type;

  public:
    const ADAS& get_adas() const;
    ADAS& get_mutable_adas();
    void set_adas(const ADAS& value);

    const Acceleration& get_acceleration() const;
    Acceleration& get_mutable_acceleration();
    void set_acceleration(const Acceleration& value);

    const AngularVelocity& get_angular_velocity() const;
    AngularVelocity& get_mutable_angular_velocity();
    void set_angular_velocity(const AngularVelocity& value);

    float get_average_speed() const;
    void set_average_speed(const float value);

    const Body& get_body() const;
    Body& get_mutable_body();
    void set_body(const Body& value);

    const Chassis& get_chassis() const;
    Chassis& get_mutable_chassis();
    void set_chassis(const Chassis& value);

    const Connectivity& get_connectivity() const;
    Connectivity& get_mutable_connectivity();
    void set_connectivity(const Connectivity& value);

    std::uint16_t get_current_overall_weight() const;
    void set_current_overall_weight(const std::uint16_t value);

    const Exterior& get_exterior() const;
    Exterior& get_mutable_exterior();
    void set_exterior(const Exterior& value);

    std::uint16_t get_height() const;
    void set_height(const std::uint16_t value);

    bool get_is_moving() const;
    void set_is_moving(const bool value);

    std::uint16_t get_length() const;
    void set_length(const std::uint16_t value);

    const MotionManagement& get_motion_management() const;
    MotionManagement& get_mutable_motion_management();
    void set_motion_management(const MotionManagement& value);

    const Powertrain& get_powertrain() const;
    Powertrain& get_mutable_powertrain();
    void set_powertrain(const Powertrain& value);

    float get_speed() const;
    void set_speed(const float value);

    const std::string get_start_time() const;
    void set_start_time(const std::string value);

    float get_traveled_distance() const;
    void set_traveled_distance(const float value);

    float get_traveled_distance_since_start() const;
    void set_traveled_distance_since_start(const float value);

    float get_trip_duration() const;
    void set_trip_duration(const float value);

    std::uint16_t get_turning_diameter() const;
    void set_turning_diameter(const std::uint16_t value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

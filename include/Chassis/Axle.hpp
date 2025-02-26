#pragma once

#include "Type.hpp"
#include <iostream>

class Wheel
{
  public:
    Wheel()          = default;
    virtual ~Wheel() = default;

  private:
    float angular_speed;
    float speed;
    std::string description;
    Type type;

  public:
    float get_angular_speed() const;
    void set_angular_speed(const float value);

    float get_speed() const;
    void set_speed(const float value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

class AxleRow
{
  public:
    AxleRow()          = default;
    virtual ~AxleRow() = default;

  private:
    std::uint16_t axle_width;
    float steering_angle;
    std::uint8_t tire_aspect_ratio;
    float tire_diameter;
    std::uint16_t tire_width;
    std::uint16_t track_width;
    std::uint16_t tread_width;
    Wheel wheel_right;
    Wheel wheel_left;
    std::uint8_t wheel_count;
    float wheel_diameter;
    float wheel_width;
    std::string description;
    Type type;

  public:
    const std::uint16_t get_axle_width() const;
    std::uint16_t get_mutable_axle_width();
    void set_axle_width(const std::uint16_t value);

    float get_steering_angle() const;
    void set_steering_angle(const float value);

    const std::uint8_t get_tire_aspect_ratio() const;
    std::uint8_t get_mutable_tire_aspect_ratio();
    void set_tire_aspect_ratio(const std::uint8_t value);

    float get_tire_diameter() const;
    void set_tire_diameter(const float value);

    const std::uint16_t get_tire_width() const;
    std::uint16_t get_mutable_tire_width();
    void set_tire_width(const std::uint16_t value);

    const std::uint16_t get_track_width() const;
    std::uint16_t get_mutable_track_width();
    void set_track_width(const std::uint16_t value);

    const std::uint16_t get_tread_width() const;
    std::uint16_t get_mutable_tread_width();
    void set_tread_width(const std::uint16_t value);

    const Wheel& get_wheel_right() const;
    Wheel& get_mutable_wheel_right();
    void set_wheel_right(const Wheel& value);

    const Wheel& get_wheel_left() const;
    Wheel& get_mutable_wheel_left();
    void set_wheel_left(const Wheel& value);

    std::uint8_t get_wheel_count() const;
    void set_wheel_count(const std::uint8_t value);

    float get_wheel_diameter() const;
    void set_wheel_diameter(const float value);

    float get_wheel_width() const;
    void set_wheel_width(const float value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};

class Axle
{
  public:
    Axle()          = default;
    virtual ~Axle() = default;

  private:
    AxleRow row1;
    AxleRow row2;
    std::string description;
    Type type;

  public:
    const AxleRow& get_row1() const;
    AxleRow& get_mutable_row1();
    void set_row1(const AxleRow& value);

    const AxleRow& get_row2() const;
    AxleRow& get_mutable_row2();
    void set_row2(const AxleRow& value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};
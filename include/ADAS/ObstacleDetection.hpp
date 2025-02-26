#pragma once

#include "Type.hpp"
#include <iostream>

class Detection
{
  public:
    Detection()          = default;
    virtual ~Detection() = default;

  private:
    float distance;
    bool is_enabled;
    bool is_error;
    bool is_warning;
    std::uint32_t time_gap;
    std::string warning_type;

  public:
    const float get_distance() const;
    float get_mutable_distance();
    void set_distance(const float value);

    const bool get_is_enabled() const;
    bool get_mutable_is_enabled();
    void set_is_enabled(const bool value);

    const bool get_is_error() const;
    bool get_mutable_is_error();
    void set_is_error(const bool value);

    const bool get_is_warning() const;
    bool get_mutable_is_warning();
    void set_is_warning(const bool value);

    const std::uint32_t get_time_gap() const;
    std::uint32_t get_mutable_time_gap();
    void set_time_gap(const std::uint32_t value);

    const std::string& get_warning_type() const;
    std::string& get_mutable_warning_type();
    void set_warning_type(const std::string& value);
};

class ObstacleDetection
{
  public:
    ObstacleDetection()          = default;
    virtual ~ObstacleDetection() = default;

  private:
    Detection front;
    Detection rear;
    Detection center;
    Detection left;
    Detection right;
    std::string description;
    Type type;

  public:
    const Detection& get_front() const;
    Detection& get_mutable_front();
    void set_front(const Detection& value);

    const Detection& get_rear() const;
    Detection& get_mutable_rear();
    void set_rear(const Detection& value);

    const Detection& get_center() const;
    Detection& get_mutable_center();
    void set_center(const Detection& value);

    const Detection& get_left() const;
    Detection& get_mutable_left();
    void set_left(const Detection& value);

    const Detection& get_right() const;
    Detection& get_mutable_right();
    void set_right(const Detection& value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};
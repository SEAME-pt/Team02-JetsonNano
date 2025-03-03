#pragma once

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
    float get_distance() const;
    void set_distance(const float value);

    bool get_is_enabled() const;
    void set_is_enabled(const bool value);

    bool get_is_error() const;
    void set_is_error(const bool value);

    bool get_is_warning() const;
    void set_is_warning(const bool value);

    std::uint32_t get_time_gap() const;
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

  public:
    const Detection& get_front() const;
    Detection& get_mutable_front();
    void set_front(const Detection& value);

    const Detection& get_rear() const;
    Detection& get_mutable_rear();
    void set_rear(const Detection& value);
};
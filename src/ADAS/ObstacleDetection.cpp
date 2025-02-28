#include "ObstacleDetection.hpp"

float Detection::get_distance() const
{
    return distance;
}

void Detection::set_distance(const float value)
{
    this->distance = value;
}

bool Detection::get_is_enabled() const
{
    return is_enabled;
}

void Detection::set_is_enabled(const bool value)
{
    this->is_enabled = value;
}

bool Detection::get_is_error() const
{
    return is_error;
}

void Detection::set_is_error(const bool value)
{
    this->is_error = value;
}

bool Detection::get_is_warning() const
{
    return is_warning;
}

void Detection::set_is_warning(const bool value)
{
    this->is_warning = value;
}

std::uint32_t Detection::get_time_gap() const
{
    return time_gap;
}

void Detection::set_time_gap(const std::uint32_t value)
{
    this->time_gap = value;
}

const std::string& Detection::get_warning_type() const
{
    return warning_type;
}
std::string& Detection::get_mutable_warning_type()
{
    return warning_type;
}
void Detection::set_warning_type(const std::string& value)
{
    this->warning_type = value;
}

const Detection& ObstacleDetection::get_front() const
{
    return front;
}
Detection& ObstacleDetection::get_mutable_front()
{
    return front;
}
void ObstacleDetection::set_front(const Detection& value)
{
    this->front = value;
}

const Detection& ObstacleDetection::get_rear() const
{
    return rear;
}
Detection& ObstacleDetection::get_mutable_rear()
{
    return rear;
}
void ObstacleDetection::set_rear(const Detection& value)
{
    this->rear = value;
}

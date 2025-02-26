#include "ObstacleDetection.hpp"

Detection::Detection() {};

const float Detection::get_distance() const
{
    return distance;
}
float Detection::get_mutable_distance()
{
    return distance;
}
void Detection::set_distance(const float value)
{
    this->distance = value;
}

const bool Detection::get_is_enabled() const
{
    return is_enabled;
}
bool Detection::get_mutable_is_enabled()
{
    return is_enabled;
}
void Detection::set_is_enabled(const bool value)
{
    this->is_enabled = value;
}

const bool Detection::get_is_error() const
{
    return is_error;
}
bool Detection::get_mutable_is_error()
{
    return is_error;
}
void Detection::set_is_error(const bool value)
{
    this->is_error = value;
}

const bool Detection::get_is_warning() const
{
    return is_warning;
}
bool Detection::get_mutable_is_warning()
{
    return is_warning;
}
void Detection::set_is_warning(const bool value)
{
    this->is_warning = value;
}

const std::uint32_t Detection::get_time_gap() const
{
    return time_gap;
}
std::uint32_t Detection::get_mutable_time_gap()
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

ObstacleDetection::ObstacleDetection() {}

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

const Detection& ObstacleDetection::get_center() const
{
    return center;
}
Detection& ObstacleDetection::get_mutable_center()
{
    return center;
}
void ObstacleDetection::set_center(const Detection& value)
{
    this->center = value;
}

const Detection& ObstacleDetection::get_left() const
{
    return left;
}
Detection& ObstacleDetection::get_mutable_left()
{
    return left;
}
void ObstacleDetection::set_left(const Detection& value)
{
    this->left = value;
}

const Detection& ObstacleDetection::get_right() const
{
    return right;
}
Detection& ObstacleDetection::get_mutable_right()
{
    return right;
}
void ObstacleDetection::set_right(const Detection& value)
{
    this->right = value;
}

const std::string& ObstacleDetection::get_description() const
{
    return description;
}
std::string& ObstacleDetection::get_mutable_description()
{
    return description;
}
void ObstacleDetection::set_description(const std::string& value)
{
    this->description = value;
}

Type ObstacleDetection::get_type() const
{
    return type;
}
void ObstacleDetection::set_type(Type value)
{
    this->type = value;
}
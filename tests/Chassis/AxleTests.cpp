#include "Axle.hpp"

float Wheel::get_angular_speed() const
{
    return angular_speed;
}

void Wheel::set_angular_speed(const float value)
{
    this->angular_speed = value;
}

float Wheel::get_speed() const
{
    return speed;
}

void Wheel::set_speed(const float value)
{
    this->speed = value;
}

std::uint16_t AxleRow::get_axle_width() const
{
    return axle_width;
}

void AxleRow::set_axle_width(const std::uint16_t value)
{
    this->axle_width = value;
}

float AxleRow::get_steering_angle() const
{
    return steering_angle;
}

void AxleRow::set_steering_angle(const float value)
{
    this->steering_angle = value;
}

std::uint8_t AxleRow::get_tire_aspect_ratio() const
{
    return tire_aspect_ratio;
}

void AxleRow::set_tire_aspect_ratio(const std::uint8_t value)
{
    this->tire_aspect_ratio = value;
}

float AxleRow::get_tire_diameter() const
{
    return tire_diameter;
}

void AxleRow::set_tire_diameter(const float value)
{
    this->tire_diameter = value;
}

std::uint16_t AxleRow::get_tire_width() const
{
    return tire_width;
}

void AxleRow::set_tire_width(const std::uint16_t value)
{
    this->tire_width = value;
}

std::uint16_t AxleRow::get_track_width() const
{
    return track_width;
}

void AxleRow::set_track_width(const std::uint16_t value)
{
    this->track_width = value;
}

std::uint16_t AxleRow::get_tread_width() const
{
    return tread_width;
}

void AxleRow::set_tread_width(const std::uint16_t value)
{
    this->tread_width = value;
}

const Wheel& AxleRow::get_wheel_right() const
{
    return wheel_right;
}
Wheel& AxleRow::get_mutable_wheel_right()
{
    return wheel_right;
}
void AxleRow::set_wheel_right(const Wheel& value)
{
    this->wheel_right = value;
}

const Wheel& AxleRow::get_wheel_left() const
{
    return wheel_left;
}
Wheel& AxleRow::get_mutable_wheel_left()
{
    return wheel_left;
}
void AxleRow::set_wheel_left(const Wheel& value)
{
    this->wheel_left = value;
}

std::uint8_t AxleRow::get_wheel_count() const
{
    return wheel_count;
}

void AxleRow::set_wheel_count(const std::uint8_t value)
{
    this->wheel_count = value;
}

float AxleRow::get_wheel_diameter() const
{
    return wheel_diameter;
}

void AxleRow::set_wheel_diameter(const float value)
{
    this->wheel_diameter = value;
}

float AxleRow::get_wheel_width() const
{
    return wheel_width;
}

void AxleRow::set_wheel_width(const float value)
{
    this->wheel_width = value;
}

const AxleRow& Axle::get_row1() const
{
    return row1;
}
AxleRow& Axle::get_mutable_row1()
{
    return row1;
}
void Axle::set_row1(const AxleRow& value)
{
    this->row1 = value;
}

const AxleRow& Axle::get_row2() const
{
    return row2;
}
AxleRow& Axle::get_mutable_row2()
{
    return row2;
}
void Axle::set_row2(const AxleRow& value)
{
    this->row2 = value;
}

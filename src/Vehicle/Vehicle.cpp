#include "Vehicle.hpp"

const ADAS& Vehicle::get_adas() const
{
    return adas;
}
ADAS& Vehicle::get_mutable_adas()
{
    return adas;
}
void Vehicle::set_adas(const ADAS& value)
{
    this->adas = value;
}

const Acceleration& Vehicle::get_acceleration() const
{
    return acceleration;
}
Acceleration& Vehicle::get_mutable_acceleration()
{
    return acceleration;
}
void Vehicle::set_acceleration(const Acceleration& value)
{
    this->acceleration = value;
}

const AngularVelocity& Vehicle::get_angular_velocity() const
{
    return angular_velocity;
}
AngularVelocity& Vehicle::get_mutable_angular_velocity()
{
    return angular_velocity;
}
void Vehicle::set_angular_velocity(const AngularVelocity& value)
{
    this->angular_velocity = value;
}

float Vehicle::get_average_speed() const
{
    return average_speed;
}
void Vehicle::set_average_speed(const float value)
{
    this->average_speed = value;
}

const Body& Vehicle::get_body() const
{
    return body;
}
Body& Vehicle::get_mutable_body()
{
    return body;
}
void Vehicle::set_body(const Body& value)
{
    this->body = value;
}

const Chassis& Vehicle::get_chassis() const
{
    return chassis;
}
Chassis& Vehicle::get_mutable_chassis()
{
    return chassis;
}
void Vehicle::set_chassis(const Chassis& value)
{
    this->chassis = value;
}

const Connectivity& Vehicle::get_connectivity() const
{
    return connectivity;
}
Connectivity& Vehicle::get_mutable_connectivity()
{
    return connectivity;
}
void Vehicle::set_connectivity(const Connectivity& value)
{
    this->connectivity = value;
}

std::uint16_t Vehicle::get_current_overall_weight() const
{
    return current_overall_weight;
}
void Vehicle::set_current_overall_weight(const std::uint16_t value)
{
    this->current_overall_weight = value;
}

const Exterior& Vehicle::get_exterior() const
{
    return exterior;
}
Exterior& Vehicle::get_mutable_exterior()
{
    return exterior;
}
void Vehicle::set_exterior(const Exterior& value)
{
    this->exterior = value;
}

std::uint16_t Vehicle::get_height() const
{
    return height;
}
void Vehicle::set_height(const std::uint16_t value)
{
    this->height = value;
}

bool Vehicle::get_is_moving() const
{
    return is_moving;
}
void Vehicle::set_is_moving(const bool value)
{
    this->is_moving = value;
}

std::uint16_t Vehicle::get_length() const
{
    return length;
}
void Vehicle::set_length(const std::uint16_t value)
{
    this->length = value;
}

const MotionManagement& Vehicle::get_motion_management() const
{
    return motion_management;
}
MotionManagement& Vehicle::get_mutable_motion_management()
{
    return motion_management;
}
void Vehicle::set_motion_management(const MotionManagement& value)
{
    this->motion_management = value;
}

const Powertrain& Vehicle::get_powertrain() const
{
    return powertrain;
}
Powertrain& Vehicle::get_mutable_powertrain()
{
    return powertrain;
}
void Vehicle::set_powertrain(const Powertrain& value)
{
    this->powertrain = value;
}

float Vehicle::get_speed() const
{
    return speed;
}
void Vehicle::set_speed(const float value)
{
    this->speed = value;
}

const std::string Vehicle::get_start_time() const
{
    return start_time;
}
void Vehicle::set_start_time(const std::string value)
{
    this->start_time = value;
}

float Vehicle::get_traveled_distance() const
{
    return traveled_distance;
}
void Vehicle::set_traveled_distance(const float value)
{
    this->traveled_distance = value;
}

float Vehicle::get_traveled_distance_since_start() const
{
    return traveled_distance_since_start;
}
void Vehicle::set_traveled_distance_since_start(const float value)
{
    this->traveled_distance_since_start = value;
}

float Vehicle::get_trip_duration() const
{
    return trip_duration;
}
void Vehicle::set_trip_duration(const float value)
{
    this->trip_duration = value;
}

std::uint16_t Vehicle::get_turning_diameter() const
{
    return turning_diameter;
}
void Vehicle::set_turning_diameter(const std::uint16_t value)
{
    this->turning_diameter = value;
}

const std::string& Vehicle::get_description() const
{
    return description;
}
std::string& Vehicle::get_mutable_description()
{
    return description;
}
void Vehicle::set_description(const std::string& value)
{
    this->description = value;
}

Type Vehicle::get_type() const
{
    return type;
}
void Vehicle::set_type(Type value)
{
    this->type = value;
}

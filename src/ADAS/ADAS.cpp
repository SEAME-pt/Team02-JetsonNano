#include "ADAS.hpp"

const std::string& ADAS::get_active_autonomy_level() const
{
    return active_autonomy_level;
}
void ADAS::set_active_autonomy_level(const std::string& value)
{
    this->active_autonomy_level = value;
}

const std::string& ADAS::get_supported_autonomy_level() const
{
    return supported_autonomy_level;
}
void ADAS::set_supported_autonomy_level(const std::string& value)
{
    this->supported_autonomy_level = value;
}

const ObstacleDetection& ADAS::get_obstacle_detection() const
{
    return obstacle_detection;
}
ObstacleDetection& ADAS::get_mutable_obstacle_detection()
{
    return obstacle_detection;
}
void ADAS::set_obstacle_detection(const ObstacleDetection& value)
{
    this->obstacle_detection = value;
}

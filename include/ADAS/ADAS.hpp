#pragma once

#include "ObstacleDetection.hpp"
#include <iostream>

class ADAS
{
  public:
    ADAS()          = default;
    virtual ~ADAS() = default;

  private:
    std::string active_autonomy_level;
    std::string supported_autonomy_level;
    ObstacleDetection obstacle_detection;

  public:
    const std::string& get_active_autonomy_level() const;
    void set_active_autonomy_level(const std::string& value);

    const std::string& get_supported_autonomy_level() const;
    void set_supported_autonomy_level(const std::string& value);

    const ObstacleDetection& get_obstacle_detection() const;
    ObstacleDetection& get_mutable_obstacle_detection();
    void set_obstacle_detection(const ObstacleDetection& value);
};
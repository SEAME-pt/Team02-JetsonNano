#pragma once

#include "ObstacleDetection.hpp"
#include <iostream>

/**
 * @brief Advanced Driver Assistance System (ADAS) class
 *
 * @details This class manages the vehicle's ADAS functionalities according to
 * SAE J3016 standard. Key functionalities include:
 *          - Autonomy level management (SAE levels 0-5)
 *          - Obstacle detection systems integration
 *          - Real-time monitoring and control
 *
 * The ADAS system supports different levels of driving automation:
 * - SAE_0: No Driving Automation
 * - SAE_1: Driver Assistance
 * - SAE_2: Partial Driving Automation
 * - SAE_3: Conditional Driving Automation
 * - SAE_4: High Driving Automation
 * - SAE_5: Full Driving Automation
 *
 * @note Complies with SAE J3016_202104 standard
 * @see https://www.sae.org/standards/content/j3016_202104/
 */
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
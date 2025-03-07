#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "IVehicleObserver.hpp"

/**
 * @brief Steering wheel control and monitoring
 *
 * @details Manages steering wheel position and notifications:
 *          - Angle measurement (-180 to +180 degrees)
 *          - Observer pattern for angle changes
 *          - Real-time position updates
 *
 * @note Positive angles indicate left turn, negative indicate right turn
 * @note Implements VSS Chassis.SteeringWheel branch specification
 */
class ChassisSteeringWheel
{
  public:
    ChassisSteeringWheel()          = default;
    virtual ~ChassisSteeringWheel() = default;

  private:
    std::int16_t angle;
    std::vector<std::shared_ptr<IVehicleObserver>> observers_;

  public:
    int16_t get_angle() const;
    void set_angle(const int16_t value);
    void addObserver(std::shared_ptr<IVehicleObserver> observer);

  private:
    void notifyAngleChanged(float speed);
};

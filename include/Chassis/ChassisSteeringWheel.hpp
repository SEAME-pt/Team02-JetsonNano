#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "IVehicleObserver.hpp"

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

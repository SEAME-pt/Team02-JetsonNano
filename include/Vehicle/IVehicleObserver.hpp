#pragma once

#include <iostream>

class IVehicleObserver
{
  public:
    virtual ~IVehicleObserver()                      = default;
    virtual void onSpeedChanged(int32_t speed)       = 0;
    virtual void onSteeringAngleChanged(float angle) = 0;
};
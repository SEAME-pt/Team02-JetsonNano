#pragma once

#include "Vehicle.cpp"

class IVehicleController
{
  public:
    virtual void updateControl(Vehicle& vehicle) = 0;
    virtual ~IVehicleController()                = default;
};
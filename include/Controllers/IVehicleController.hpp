#pragma once

#include "Vehicle.cpp"

/**
 * @brief Interface for vehicle control implementations
 *
 * @details Abstract interface for implementing different vehicle control
 * strategies:
 *          - Manual control (joystick, keyboard)
 *          - Autonomous control
 *          - Remote control
 *
 * Usage:
 * - Inherit from this interface to create concrete controllers
 * - Implement updateControl() to define control behavior
 * - Register controller with vehicle system
 *
 * @note Controllers must handle their own input validation
 * @see Vehicle
 * @see XboxController
 */
class IVehicleController
{
  public:
    virtual void updateControl(Vehicle& vehicle) = 0;
    virtual ~IVehicleController()                = default;
};
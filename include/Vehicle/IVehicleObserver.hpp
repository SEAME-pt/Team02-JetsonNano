#pragma once

#include <iostream>

/**
 * @brief Vehicle observer interface for state change notifications
 *
 * @details Interface for implementing the Observer pattern to monitor vehicle
 * state changes:
 *          - Speed changes
 *          - Steering angle changes
 *
 * Used by hardware controllers and monitoring systems to react to vehicle state
 * changes.
 *
 * Usage:
 * - Inherit from this interface to create concrete observers
 * - Implement the virtual methods to handle state changes
 * - Register observers with the vehicle system
 *
 * @note All observers must implement both speed and steering angle handlers
 * @note Speed in range -100 to +100
 * @note Steering angle in degrees (-180 to +180)
 */
class IVehicleObserver
{
  public:
    virtual ~IVehicleObserver()                      = default;
    virtual void onSpeedChanged(int32_t speed)       = 0;
    virtual void onSteeringAngleChanged(float angle) = 0;
};
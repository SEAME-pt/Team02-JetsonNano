#pragma once

#include "PCA9685.hpp"

class VehicleActuator
{
  public:
    void applyVehicleState(const Vehicle& vehicle)
    {
        // Apply VSS state to hardware
        int motor_speed =
            vehicle.get_powertrain().get_electric_motor().get_speed();
        pca_.setPWM(MOTOR_CHANNEL, convertSpeedToPWM(motor_speed));
    }

  private:
    PCA9685 pca_;
};
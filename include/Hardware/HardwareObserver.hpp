#pragma once
#include "IVehicleObserver.hpp"
#include "MotorController.hpp"
#include "ServoController.hpp"
#include <memory>

class HardwareObserver : public IVehicleObserver
{
  public:
    HardwareObserver(std::shared_ptr<MotorController> motor,
                     std::shared_ptr<ServoController> servo);

    void onSpeedChanged(int32_t speed) override;

    void onSteeringAngleChanged(float angle) override;

  private:
    std::shared_ptr<MotorController> motor_controller_;
    std::shared_ptr<ServoController> servo_controller_;
};
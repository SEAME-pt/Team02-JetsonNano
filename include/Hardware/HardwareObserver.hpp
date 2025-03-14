#pragma once
#include "IVehicleObserver.hpp"
#include "MotorController.hpp"
#include "ServoController.hpp"
#include <memory>

/**
 * @brief Hardware observer for vehicle state changes
 *
 * @details Implements IVehicleObserver to control physical hardware:
 *          - Motor speed control via PCA9685
 *          - Servo angle control via PCA9685
 *          - Real-time hardware updates
 *
 * State mappings:
 * - Speed: -100 to +100 maps to motor PWM
 * - Angle: 0° to 180° maps to servo position
 *
 * @note Requires I2C communication with PCA9685
 * @see IVehicleObserver
 * @see MotorController
 * @see ServoController
 */
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
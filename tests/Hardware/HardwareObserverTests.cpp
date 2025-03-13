#include "HardwareObserver.hpp"

HardwareObserver::HardwareObserver(std::shared_ptr<MotorController> motor,
                                   std::shared_ptr<ServoController> servo)
    : motor_controller_(motor), servo_controller_(servo)
{
}

void HardwareObserver::onSpeedChanged(int32_t speed)
{
    motor_controller_->setSpeed(speed);
}

void HardwareObserver::onSteeringAngleChanged(float angle)
{
    servo_controller_->setAngle(angle);
}

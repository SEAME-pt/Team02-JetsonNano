#pragma once
#include "VehicleFactory.hpp"
#include "VSSSubscriber.hpp"
#include "VSSPublisher.hpp"
#include "MotorController.hpp"
#include "ServoController.hpp"
#include "I2C.hpp"

class VehicleSystem
{
  public:
    VehicleSystem();

    void update();

  private:
    Vehicle vehicle_;
    std::unique_ptr<VSSSubscriber> vss_subscriber_;
    std::unique_ptr<VSSPublisher> vss_publisher_;
    std::shared_ptr<I2C> i2c_;
    std::unique_ptr<MotorController> motor_controller_;
    std::unique_ptr<ServoController> servo_controller_;

    void updateHardware();
    void publish();
};
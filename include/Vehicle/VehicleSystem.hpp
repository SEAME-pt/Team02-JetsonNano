#pragma once
#include "VehicleFactory.hpp"
#include "VSSSubscriber.hpp"
#include "VSSQueryable.hpp"
#include "MotorController.hpp"
#include "ServoController.hpp"
#include "HardwareObserver.hpp"
#include "I2C.hpp"
#include "CAN.hpp"

class VehicleSystem
{
  public:
    VehicleSystem();
    VehicleSystem(const std::string& configFile);

  private:
    Vehicle vehicle_;

    std::shared_ptr<zenoh::Session> session;
    std::unique_ptr<VSSSubscriber> vss_subscriber_;
    std::unique_ptr<VSSQueryable> vss_queryable_;
    std::shared_ptr<I2C> i2c_;
    std::shared_ptr<CAN> CAN_;
    std::shared_ptr<MotorController> motor_controller_;
    std::shared_ptr<ServoController> servo_controller_;
};
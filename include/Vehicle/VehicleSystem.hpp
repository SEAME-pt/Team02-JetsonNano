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

    // Vehicle getters
    const Vehicle& getVehicle() const;
    Vehicle& getMutableVehicle();

    // Session getters
    std::shared_ptr<zenoh::Session> getSession() const;
    void setSession(const std::shared_ptr<zenoh::Session>& value);

    // VSS Subscriber getters
    const std::unique_ptr<VSSSubscriber>& getVSSSubscriber() const;
    std::unique_ptr<VSSSubscriber>& getMutableVSSSubscriber();
    void setVSSSubscriber(std::unique_ptr<VSSSubscriber> value);

    // VSS Queryable getters
    const std::unique_ptr<VSSQueryable>& getVSSQueryable() const;
    std::unique_ptr<VSSQueryable>& getMutableVSSQueryable();
    void setVSSQueryable(std::unique_ptr<VSSQueryable> value);

    // I2C getters
    std::shared_ptr<I2C> getI2C() const;
    void setI2C(const std::shared_ptr<I2C>& value);

    // CAN getters
    std::shared_ptr<CAN> getCAN() const;
    void setCAN(const std::shared_ptr<CAN>& value);

    // Motor Controller getters
    std::shared_ptr<MotorController> getMotorController() const;
    void setMotorController(const std::shared_ptr<MotorController>& value);

    // Servo Controller getters
    std::shared_ptr<ServoController> getServoController() const;
    void setServoController(const std::shared_ptr<ServoController>& value);

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
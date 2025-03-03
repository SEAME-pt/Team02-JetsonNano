#include "VehicleSystem.hpp"

VehicleSystem::VehicleSystem()
{
    vehicle_ = VehicleFactory::createDefaultVehicle();

    i2c_ = std::make_shared<I2C>();
    i2c_->init("/dev/i2c-1");
    CAN_ = std::make_shared<CAN>();
    CAN_->init("/dev/spidev0.0");

    motor_controller_ = std::make_unique<MotorController>(i2c_);
    servo_controller_ = std::make_unique<ServoController>(i2c_);

    auto hardware_observer = std::make_shared<HardwareObserver>(
        motor_controller_, servo_controller_);

    vehicle_.get_mutable_powertrain().get_mutable_electric_motor().addObserver(
        hardware_observer);

    vehicle_.get_mutable_chassis().get_mutable_steering_wheel().addObserver(
        hardware_observer);

    vss_subscriber_ = std::make_unique<VSSSubscriber>(
        vehicle_, [this](uint32_t can_id, uint8_t* data, size_t len)
        { this->CAN_->writeMessage(can_id, data, len); });

    vss_queryable_ = std::make_unique<VSSQueryable>(vehicle_);
}

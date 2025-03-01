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

    vss_subscriber_ = std::make_unique<VSSSubscriber>(vehicle_, 
        [this](uint32_t can_id, uint8_t* data, size_t len) {
            this->CAN_->writeMessage(can_id, data, len);
        });

    vss_queryable_    = std::make_unique<VSSQueryable>(vehicle_);
}

void VehicleSystem::update()
{
    updateHardware();
}

void VehicleSystem::updateHardware()
{
    // Update motor speed
    int32_t motor_speed =
        vehicle_.get_powertrain().get_electric_motor().get_speed();
    motor_controller_->setSpeed(motor_speed);

    // Update steering angle
    float steering_angle =
        vehicle_.get_chassis().get_steering_wheel().get_angle();
    servo_controller_->setAngle(steering_angle);
}

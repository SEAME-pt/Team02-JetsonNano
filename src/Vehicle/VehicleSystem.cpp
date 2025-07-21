#include "VehicleSystem.hpp"

VehicleSystem::VehicleSystem()
{
    vehicle_ = VehicleFactory::createDefaultVehicle();

    auto config = zenoh::Config::create_default();
    session     = std::make_shared<zenoh::Session>(
        zenoh::Session::open(std::move(config)));

    i2c_ = std::make_shared<I2C>();
    i2c_->init("/dev/i2c-1");

    motor_controller_ = std::make_unique<MotorController>(i2c_);
    servo_controller_ = std::make_unique<ServoController>(i2c_);

    auto hardware_observer = std::make_shared<HardwareObserver>(
        motor_controller_, servo_controller_);

    vehicle_.get_mutable_powertrain().get_mutable_electric_motor().addObserver(
        hardware_observer);

    vehicle_.get_mutable_chassis().get_mutable_steering_wheel().addObserver(
        hardware_observer);

    vss_subscriber_ = std::make_unique<VSSSubscriber>(vehicle_, session);

    vss_queryable_ = std::make_unique<VSSQueryable>(vehicle_, session);
}

VehicleSystem::VehicleSystem(const std::string& configFile)
{
    vehicle_ = VehicleFactory::createDefaultVehicle();

    auto config = zenoh::Config::from_file(configFile);
    session     = std::make_shared<zenoh::Session>(
        zenoh::Session::open(std::move(config)));

    i2c_ = std::make_shared<I2C>();
    i2c_->init("/dev/i2c-1");

    motor_controller_ = std::make_unique<MotorController>(i2c_);
    servo_controller_ = std::make_unique<ServoController>(i2c_);

    auto hardware_observer = std::make_shared<HardwareObserver>(
        motor_controller_, servo_controller_);

    vehicle_.get_mutable_powertrain().get_mutable_electric_motor().addObserver(
        hardware_observer);

    vehicle_.get_mutable_chassis().get_mutable_steering_wheel().addObserver(
        hardware_observer);

    vss_subscriber_ = std::make_unique<VSSSubscriber>(vehicle_, session);

    vss_queryable_ = std::make_unique<VSSQueryable>(vehicle_, session);
}

const Vehicle& VehicleSystem::getVehicle() const
{
    return vehicle_;
}
Vehicle& VehicleSystem::getMutableVehicle()
{
    return vehicle_;
}

std::shared_ptr<zenoh::Session> VehicleSystem::getSession() const
{
    return session;
}
void VehicleSystem::setSession(const std::shared_ptr<zenoh::Session>& value)
{
    session = value;
}

const std::unique_ptr<VSSSubscriber>& VehicleSystem::getVSSSubscriber() const
{
    return vss_subscriber_;
}
std::unique_ptr<VSSSubscriber>& VehicleSystem::getMutableVSSSubscriber()
{
    return vss_subscriber_;
}
void VehicleSystem::setVSSSubscriber(std::unique_ptr<VSSSubscriber> value)
{
    vss_subscriber_ = std::move(value);
}

const std::unique_ptr<VSSQueryable>& VehicleSystem::getVSSQueryable() const
{
    return vss_queryable_;
}
std::unique_ptr<VSSQueryable>& VehicleSystem::getMutableVSSQueryable()
{
    return vss_queryable_;
}
void VehicleSystem::setVSSQueryable(std::unique_ptr<VSSQueryable> value)
{
    vss_queryable_ = std::move(value);
}

std::shared_ptr<I2C> VehicleSystem::getI2C() const
{
    return i2c_;
}
void VehicleSystem::setI2C(const std::shared_ptr<I2C>& value)
{
    i2c_ = value;
}

std::shared_ptr<MotorController> VehicleSystem::getMotorController() const
{
    return motor_controller_;
}
void VehicleSystem::setMotorController(
    const std::shared_ptr<MotorController>& value)
{
    motor_controller_ = value;
}

std::shared_ptr<ServoController> VehicleSystem::getServoController() const
{
    return servo_controller_;
}
void VehicleSystem::setServoController(
    const std::shared_ptr<ServoController>& value)
{
    servo_controller_ = value;
}
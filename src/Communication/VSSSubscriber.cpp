#include "VSSSubscriber.hpp"

VSSSubscriber::VSSSubscriber(Vehicle& vehicle) : vehicle_(vehicle)
{
    auto config = zenoh::Config::create_default();
    session     = std::make_unique<zenoh::Session>(
        zenoh::Session::open(std::move(config)));

    setupSubscriptions();
}

void VSSSubscriber::setupSubscriptions()
{
    // Initialize subscribers
    throttle_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Powertrain/ElectricMotor/Speed",
        [this](const zenoh::Sample& sample)
        {
            float throttle = std::stof(sample.get_payload().as_string());
            this->vehicle_.get_mutable_powertrain()
                .get_mutable_electric_motor()
                .set_speed(throttle);
        },
        zenoh::closures::none));

    steering_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Chassis/SteeringWheel/Angle",
        [this](const zenoh::Sample& sample)
        {
            float steering = std::stof(sample.get_payload().as_string());
            this->vehicle_.get_mutable_chassis()
                .get_mutable_steering_wheel()
                .set_angle(steering);
        },
        zenoh::closures::none));
}
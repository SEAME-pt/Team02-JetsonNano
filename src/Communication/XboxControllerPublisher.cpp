#include <XboxControllerPublisher.hpp>

XboxControllerPublisher::XboxControllerPublisher()
{
    auto config = zenoh::Config::create_default();
    session     = std::make_unique<zenoh::Session>(
        zenoh::Session::open(std::move(config)));

    throttle_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Powertrain/ElectricMotor/Speed")));
    steering_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Chassis/SteeringWheel/Angle")));
}

void XboxControllerPublisher::publishSpeed(float speed)
{
    throttle_pub->put(std::to_string(speed));
}

void XboxControllerPublisher::publishSteering(float steering)
{
    steering_pub->put(std::to_string(steering));
}

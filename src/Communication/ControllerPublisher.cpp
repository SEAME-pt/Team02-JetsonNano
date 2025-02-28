#include <ControllerPublisher.hpp>

ControllerPublisher::ControllerPublisher()
{
    auto config = zenoh::Config::create_default();
    session     = std::make_unique<zenoh::Session>(
        zenoh::Session::open(std::move(config)));

    throttle_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Powertrain/ElectricMotor/Speed")));
    steering_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Chassis/SteeringWheel/Angle")));
    beamLow_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Beam/Low")));
    beamHigh_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Beam/High")));
    running_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Running")));
    parking_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Parking")));
    fogRear_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Fog/Rear")));
    fogFront_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Fog/Front")));
    brake_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Brake")));
    hazard_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Hazard")));
    directionIndicatorLeft_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/DirectionIndicator/Left")));
    directionIndicatorRight_pub.emplace(session->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/DirectionIndicator/Right")));
}

void ControllerPublisher::publishSpeed(float speed)
{
    throttle_pub->put(std::to_string(speed));
}

void ControllerPublisher::publishSteering(float steering)
{
    steering_pub->put(std::to_string(steering));
}

void ControllerPublisher::publishBeamLow(bool isOn)
{
    beamLow_pub->put(std::to_string(isOn));
}

void ControllerPublisher::publishBeamHigh(bool isOn)
{
    beamHigh_pub->put(std::to_string(isOn));
}

void ControllerPublisher::publishRunning(bool isOn)
{
    running_pub->put(std::to_string(isOn));
}

void ControllerPublisher::publishParking(bool isOn)
{
    parking_pub->put(std::to_string(isOn));
}

void ControllerPublisher::publishFogRear(bool isOn)
{
    fogRear_pub->put(std::to_string(isOn));
}

void ControllerPublisher::publishFogFront(bool isOn)
{
    fogFront_pub->put(std::to_string(isOn));
}

void ControllerPublisher::publishBrake(bool isActive)
{
    brake_pub->put(std::to_string(isActive));
}

void ControllerPublisher::publishHazard(bool isSignaling)
{
    hazard_pub->put(std::to_string(isSignaling));
}

void ControllerPublisher::publishDirectionIndicatorLeft(bool isSignaling)
{
    directionIndicatorLeft_pub->put(std::to_string(isSignaling));
}

void ControllerPublisher::publishDirectionIndicatorRight(bool isSignaling)
{
    directionIndicatorRight_pub->put(std::to_string(isSignaling));
}

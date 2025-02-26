#include <VSSPublisher.hpp>

VSSPublisher::VSSPublisher(Vehicle& vehicle) : vehicle_(vehicle)
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
#include <VSSPublisher.hpp>

VSSPublisher::VSSPublisher(Vehicle& vehicle) : vehicle_(vehicle)
{
    auto config = zenoh::Config::create_default();

    session = std::make_unique<zenoh::Session>(
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

void VSSPublisher::monitor()
{
    while (true)
    {
        // Publish vehicle speed
        float speed =
            vehicle_.get_powertrain().get_electric_motor().get_speed();
        throttle_pub->put(std::to_string(speed));

        // Publish steering angle
        float angle = vehicle_.get_chassis().get_steering_wheel().get_angle();
        steering_pub->put(std::to_string(angle));

        // Publish lights status
        auto lights = vehicle_.get_body().get_lights();
        beamLow_pub->put(std::to_string(lights.get_beam_low()));
        beamHigh_pub->put(std::to_string(lights.get_beam_high()));
        running_pub->put(std::to_string(lights.get_running()));
        parking_pub->put(std::to_string(lights.get_parking()));
        fogRear_pub->put(std::to_string(lights.get_fog_rear()));
        fogFront_pub->put(std::to_string(lights.get_fog_front()));
        brake_pub->put(std::to_string(lights.get_brake()));
        hazard_pub->put(std::to_string(lights.get_hazard()));
        directionIndicatorLeft_pub->put(
            std::to_string(lights.get_direction_left()));
        directionIndicatorRight_pub->put(
            std::to_string(lights.get_direction_right()));

        // Sleep for a short duration to control publish rate
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
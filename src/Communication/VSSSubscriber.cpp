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

    beamLow_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/Beam/Low",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_beam_low(
                value);
        },
        zenoh::closures::none));

    beamHigh_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/Beam/High",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body()
                .get_mutable_lights()
                .set_beam_high(value);
        },
        zenoh::closures::none));

    running_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/Running",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_running(
                value);
        },
        zenoh::closures::none));

    parking_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/Parking",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_parking(
                value);
        },
        zenoh::closures::none));

    fogRear_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/Fog/Rear",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_fog_rear(
                value);
        },
        zenoh::closures::none));

    fogFront_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/Fog/Front",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body()
                .get_mutable_lights()
                .set_fog_front(value);
        },
        zenoh::closures::none));

    brake_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/Brake",
        [this](const zenoh::Sample& sample)
        {
            BrakeLights value;
            bool isActive = std::stoi(sample.get_payload().as_string());

            value.set_is_active(isActive);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_brake(
                value);
        },
        zenoh::closures::none));

    hazard_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/Hazard",
        [this](const zenoh::Sample& sample)
        {
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            value.set_is_signaling(isSignaling);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_hazard(
                value);
        },
        zenoh::closures::none));
    directionIndicatorLeft_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/DirectionIndicator/Left",
        [this](const zenoh::Sample& sample)
        {
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            value.set_is_signaling(isSignaling);
            this->vehicle_.get_mutable_body()
                .get_mutable_lights()
                .set_direction_indicator_left(value);
        },
        zenoh::closures::none));

    directionIndicatorRight_subscriber.emplace(session->declare_subscriber(
        "Vehicle/1/Body/Lights/DirectionIndicator/Right",
        [this](const zenoh::Sample& sample)
        {
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            value.set_is_signaling(isSignaling);
            this->vehicle_.get_mutable_body()
                .get_mutable_lights()
                .set_direction_indicator_right(value);
        },
        zenoh::closures::none));
}
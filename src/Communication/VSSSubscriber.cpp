#include "VSSSubscriber.hpp"

VSSSubscriber::VSSSubscriber(Vehicle& vehicle,
                             std::shared_ptr<zenoh::Session> session)
    : vehicle_(vehicle)
{
    sendToCAN_ = [](uint32_t, uint8_t*, size_t) {};
    session_   = session;

    setupSubscriptions();
}

VSSSubscriber::VSSSubscriber(
    Vehicle& vehicle, std::function<void(uint32_t, uint8_t*, size_t)> sendToCAN,
    std::shared_ptr<zenoh::Session> session)
    : vehicle_(vehicle), sendToCAN_(sendToCAN)
{
    session_ = session;

    setupSubscriptions();
}

void VSSSubscriber::setupSubscriptions()
{
    throttle_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Powertrain/ElectricMotor/Speed",
        [this](const zenoh::Sample& sample)
        {
            float throttle = std::stof(sample.get_payload().as_string());
            this->vehicle_.get_mutable_powertrain()
                .get_mutable_electric_motor()
                .set_speed(throttle);
        },
        zenoh::closures::none));

    steering_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Chassis/SteeringWheel/Angle",
        [this](const zenoh::Sample& sample)
        {
            float steering = std::stof(sample.get_payload().as_string());
            this->vehicle_.get_mutable_chassis()
                .get_mutable_steering_wheel()
                .set_angle(steering);
        },
        zenoh::closures::none));

    beamLow_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Beam/Low",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_beam_low(
                value);

            std::cout << "Low" << std::endl;
            lights_[0] ^= (1 << 2);
            this->sendToCAN_(0x03, lights_, sizeof(lights_));
        },
        zenoh::closures::none));

    beamHigh_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Beam/High",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body()
                .get_mutable_lights()
                .set_beam_high(value);

            std::cout << "High" << std::endl;
            lights_[0] ^= (1 << 3);
            this->sendToCAN_(0x03, lights_, sizeof(lights_));
        },
        zenoh::closures::none));

    running_subscriber.emplace(session_->declare_subscriber(
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

    parking_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Parking",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_parking(
                value);

            std::cout << "Parking" << std::endl;
            lights_[0] ^= (1 << 7);
            this->sendToCAN_(0x03, lights_, sizeof(lights_));
        },
        zenoh::closures::none));

    fogRear_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Fog/Rear",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_fog_rear(
                value);

            std::cout << "RearFog" << std::endl;
            lights_[0] ^= (1 << 5);
            this->sendToCAN_(0x03, lights_, sizeof(lights_));
        },
        zenoh::closures::none));

    fogFront_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Fog/Front",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            value.set_is_on(isOn);
            this->vehicle_.get_mutable_body()
                .get_mutable_lights()
                .set_fog_front(value);

            std::cout << "FrontFog" << std::endl;
            lights_[0] ^= (1 << 4);
            this->sendToCAN_(0x03, lights_, sizeof(lights_));
        },
        zenoh::closures::none));

    brake_subscriber.emplace(session_->declare_subscriber(
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

    hazard_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Hazard",
        [this](const zenoh::Sample& sample)
        {
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            value.set_is_signaling(isSignaling);
            this->vehicle_.get_mutable_body().get_mutable_lights().set_hazard(
                value);

            std::cout << "Hazard" << std::endl;
            lights_[0] ^= (1 << 6);
            this->sendToCAN_(0x03, lights_, sizeof(lights_));
        },
        zenoh::closures::none));
    directionIndicatorLeft_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/DirectionIndicator/Left",
        [this](const zenoh::Sample& sample)
        {
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            value.set_is_signaling(isSignaling);
            this->vehicle_.get_mutable_body()
                .get_mutable_lights()
                .set_direction_indicator_left(value);

            std::cout << "Left" << std::endl;
            lights_[0] ^= (1 << 1);
            if (((lights_[0] >> 0) & 1) == 1)
                lights_[0] ^= (1 << 0);
            this->sendToCAN_(0x03, lights_, sizeof(lights_));
        },
        zenoh::closures::none));

    directionIndicatorRight_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/DirectionIndicator/Right",
        [this](const zenoh::Sample& sample)
        {
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            value.set_is_signaling(isSignaling);
            this->vehicle_.get_mutable_body()
                .get_mutable_lights()
                .set_direction_indicator_right(value);

            // CAN communication
            std::cout << "Right" << std::endl;
            lights_[0] ^= (1 << 0);
            if (((lights_[0] >> 1) & 1) == 1)
                lights_[0] ^= (1 << 1);
            this->sendToCAN_(0x03, lights_, sizeof(lights_));
        },
        zenoh::closures::none));

    speed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed",
        [this](const zenoh::Sample& sample)
        {
            float speed = std::stof(sample.get_payload().as_string());
            this->vehicle_.set_speed(speed);
        },
        zenoh::closures::none));

    stateOfCharge_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Powertrain/TractionBattery/StateOfCharge",
        [this](const zenoh::Sample& sample)
        {
            float stateOfCharge = std::stof(sample.get_payload().as_string());
            this->vehicle_.get_mutable_powertrain()
                .get_mutable_traction_battery()
                .set_state_of_charge_displayed(stateOfCharge);
        },
        zenoh::closures::none));

    currentVoltage_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Powertrain/TractionBattery/CurrentVoltage",
        [this](const zenoh::Sample& sample)
        {
            float currentVoltage = std::stof(sample.get_payload().as_string());
            this->vehicle_.get_mutable_powertrain()
                .get_mutable_traction_battery()
                .set_current_voltage(currentVoltage);
        },
        zenoh::closures::none));

    currentCurrent_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Powertrain/TractionBattery/CurrentCurrent",
        [this](const zenoh::Sample& sample)
        {
            float currentCurrent = std::stof(sample.get_payload().as_string());
            this->vehicle_.get_mutable_powertrain()
                .get_mutable_traction_battery()
                .set_current_current(currentCurrent);
        },
        zenoh::closures::none));

    currentPower_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Powertrain/TractionBattery/CurrentPower",
        [this](const zenoh::Sample& sample)
        {
            float currentPower = std::stof(sample.get_payload().as_string());
            this->vehicle_.get_mutable_powertrain()
                .get_mutable_traction_battery()
                .set_current_power(currentPower);
        },
        zenoh::closures::none));
    currentGear_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Powertrain/Transmission/CurrentGear",
        [this](const zenoh::Sample& sample)
        {
            int8_t currentGear = std::stoi(sample.get_payload().as_string());
            this->vehicle_.get_mutable_powertrain()
                .get_mutable_transmission()
                .set_current_gear(currentGear);
            uint8_t gearData[1];
            gearData[0] = static_cast<uint8_t>(currentGear);
            this->sendToCAN_(0x04, gearData, sizeof(gearData));
        },
        zenoh::closures::none));
}
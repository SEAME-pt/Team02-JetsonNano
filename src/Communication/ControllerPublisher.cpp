#include <ControllerPublisher.hpp>

ControllerPublisher::ControllerPublisher(
    std::shared_ptr<zenoh::Session> session)
    : session_(session),
      provider_(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})))
{
    throttle_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Powertrain/ElectricMotor/Speed")));
    steering_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Chassis/SteeringWheel/Angle")));
    beamLow_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Beam/Low")));
    beamHigh_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Beam/High")));
    running_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Running")));
    parking_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Parking")));
    fogRear_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Fog/Rear")));
    fogFront_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Fog/Front")));
    brake_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Brake")));
    hazard_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Hazard")));
    directionIndicatorLeft_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/DirectionIndicator/Left")));
    directionIndicatorRight_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/DirectionIndicator/Right")));
    currentGear_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Powertrain/Transmission/CurrentGear")));
}

void ControllerPublisher::publishSpeed(float speed)
{
    string value_str = to_string(speed);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    throttle_pub->put(std::move(buf));
}

void ControllerPublisher::publishSteering(float steering)
{
    string value_str = to_string(steering);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    steering_pub->put(std::move(buf));
}

void ControllerPublisher::publishBeamLow(bool isOn)
{
    string value_str = to_string(isOn);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    beamLow_pub->put(std::move(buf));
}

void ControllerPublisher::publishBeamHigh(bool isOn)
{
    string value_str = to_string(isOn);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    beamHigh_pub->put(std::move(buf));
}

void ControllerPublisher::publishRunning(bool isOn)
{
    string value_str = to_string(isOn);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    running_pub->put(std::move(buf));
}

void ControllerPublisher::publishParking(bool isOn)
{
    string value_str = to_string(isOn);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    parking_pub->put(std::move(buf));
}

void ControllerPublisher::publishFogRear(bool isOn)
{
    string value_str = to_string(isOn);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    fogRear_pub->put(std::move(buf));
}

void ControllerPublisher::publishFogFront(bool isOn)
{
    string value_str = to_string(isOn);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    fogFront_pub->put(std::move(buf));
}

void ControllerPublisher::publishBrake(bool)
{
    string value_str = to_string(isActive);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    brake_pub->put(std::move(buf));
}

void ControllerPublisher::publishHazard(bool isSignaling)
{
    string value_str = to_string(isSignaling);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    hazard_pub->put(std::move(buf));
}

void ControllerPublisher::publishDirectionIndicatorLeft(bool isSignaling)
{
    string value_str = to_string(isSignaling);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    directionIndicatorLeft_pub->put(std::move(buf));
}

void ControllerPublisher::publishDirectionIndicatorRight(bool isSignaling)
{
    string value_str = to_string(isSignaling);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    directionIndicatorRight_pub->put(std::move(buf));
}

void ControllerPublisher::publishCurrentGear(int gear)
{
    string value_str = to_string(gear);
    const auto len   = value_str.size() + 1;
    auto alloc_result =
        provider.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    currentGear_pub->put(std::move(buf));
}

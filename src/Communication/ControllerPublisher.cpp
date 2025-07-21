#include <ControllerPublisher.hpp>

ControllerPublisher::ControllerPublisher(
    std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
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
    activeAutonomyLevel_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/ADAS/ActiveAutonomyLevel")));

    mpcTrajectory_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/ADAS/MPC/Trajectory")));

    desiredSpeed_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/ADAS/speedPid/DesiredSpeed")));

    laneAlert_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/ADAS/LaneAlert")));

    SAElevelError_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/ADAS/SAELevelAttributionError")));
}

void ControllerPublisher::publishSpeed(float speed)
{
    std::string value_str = std::to_string(speed);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    throttle_pub->put(std::move(buf));
}

void ControllerPublisher::publishSteering(float steering)
{
    std::string value_str = std::to_string(steering);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    steering_pub->put(std::move(buf));
}

void ControllerPublisher::publishBeamLow(bool isOn)
{
    std::string value_str = std::to_string(isOn);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    beamLow_pub->put(std::move(buf));
}

void ControllerPublisher::publishBeamHigh(bool isOn)
{
    std::string value_str = std::to_string(isOn);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    beamHigh_pub->put(std::move(buf));
}

void ControllerPublisher::publishRunning(bool isOn)
{
    std::string value_str = std::to_string(isOn);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    running_pub->put(std::move(buf));
}

void ControllerPublisher::publishParking(bool isOn)
{
    std::string value_str = std::to_string(isOn);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    parking_pub->put(std::move(buf));
}

void ControllerPublisher::publishFogRear(bool isOn)
{
    std::string value_str = std::to_string(isOn);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    fogRear_pub->put(std::move(buf));
}

void ControllerPublisher::publishFogFront(bool isOn)
{
    std::string value_str = std::to_string(isOn);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    fogFront_pub->put(std::move(buf));
}

void ControllerPublisher::publishBrake(bool isActive)
{
    std::string value_str = std::to_string(isActive);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    brake_pub->put(std::move(buf));
}

void ControllerPublisher::publishHazard(bool isSignaling)
{
    std::string value_str = std::to_string(isSignaling);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    hazard_pub->put(std::move(buf));
}

void ControllerPublisher::publishDirectionIndicatorLeft(bool isSignaling)
{
    std::string value_str = std::to_string(isSignaling);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    directionIndicatorLeft_pub->put(std::move(buf));
}

void ControllerPublisher::publishDirectionIndicatorRight(bool isSignaling)
{
    std::string value_str = std::to_string(isSignaling);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    directionIndicatorRight_pub->put(std::move(buf));
}

void ControllerPublisher::publishCurrentGear(int gear)
{
    std::string value_str = std::to_string(gear);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    currentGear_pub->put(std::move(buf));
}

void ControllerPublisher::publishActiveAutonomyLevel(std::string level)
{
    std::string value_str = level;
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    activeAutonomyLevel_pub->put(std::move(buf));
}

void ControllerPublisher::publishMpcTrajectory(const std::string trajectory)
{
    std::string value_str = trajectory;
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    mpcTrajectory_pub->put(std::move(buf));
}

void ControllerPublisher::publishDesiredSpeed(float speed)
{
    std::string value_str = std::to_string(speed);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    desiredSpeed_pub->put(std::move(buf));
}

void ControllerPublisher::publishLaneAlert(std::string lane)
{
    const auto len = lane.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), lane.c_str(), len);
    laneAlert_pub->put(std::move(buf));
}

void ControllerPublisher::publishSAELevelAttributionError(std::string level)
{
    const auto len = level.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), level.c_str(), len);
    SAElevelError_pub->put(std::move(buf));
}
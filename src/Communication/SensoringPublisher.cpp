#include <SensoringPublisher.hpp>

SensoringPublisher::SensoringPublisher(std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    speed_pub.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/Speed"),
    zenoh::PublisherOptions{
        .priority = zenoh::Priority::Z_PRIORITY_REAL_TIME
    }));
    current_voltage_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Powertrain/TractionBattery/CurrentVoltage")));
    current_current_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Powertrain/TractionBattery/CurrentCurrent")));
    current_power_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Powertrain/TractionBattery/CurrentPower")));
    state_of_charge_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Powertrain/TractionBattery/StateOfCharge")));
}

void SensoringPublisher::publishSpeed(float speed)
{
    std::string value_str = std::to_string(speed);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    speed_pub->put(std::move(buf));
}

void SensoringPublisher::publishCurrentVoltage(float voltage)
{
    std::string value_str = std::to_string(voltage);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    current_voltage_pub->put(std::move(buf));
}

void SensoringPublisher::publishCurrentCurrent(float current)
{
    std::string value_str = std::to_string(current);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    current_current_pub->put(std::move(buf));
}

void SensoringPublisher::publishCurrentPower(float power)
{
    std::string value_str = std::to_string(power);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    current_power_pub->put(std::move(buf));
}

void SensoringPublisher::publishStateOfCharge(float state_of_charge)
{
    std::string value_str = std::to_string(state_of_charge);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    state_of_charge_pub->put(std::move(buf));
}

#include <Publisher.hpp>

Publisher::Publisher(std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    SpeedKp_pub.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/speedpid/kp")));
    SpeedKi_pub.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/speedpid/ki")));
    SpeedKd_pub.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/speedpid/kd")));
}

void Publisher::publishSpeedKp(float speed)
{
    std::string value_str = std::to_string(speed);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    SpeedKp_pub->put(std::move(buf));
}

void Publisher::publishSpeedKi(float speed)
{
    std::string value_str = std::to_string(speed);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    SpeedKi_pub->put(std::move(buf));
}

void Publisher::publishSpeedKd(float speed)
{
    std::string value_str = std::to_string(speed);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    SpeedKd_pub->put(std::move(buf));
}
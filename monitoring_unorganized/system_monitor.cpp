#include "system_monitor.hpp"

SystemMonitor::SystemMonitor(std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));

    cpuLoad_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/SystemMonitor/cpuLoad")));

    cpuUsage_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/SystemMonitor/cpuUsage")));

    memory_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/SystemMonitor/memory")));

    temperature_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/SystemMonitor/temperature")));

    gpuUsage_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/SystemMonitor/gpuUsage")));
}

void SystemMonitor::publishCpuLoad(float cpuLoad)
{
    std::string value_str = std::to_string(cpuLoad);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    cpuLoad_pub->put(std::move(buf));
}

void SystemMonitor::publishCpuUsage(float cpuUsage)
{
    std::string value_str = std::to_string(cpuUsage);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    cpuUsage_pub->put(std::move(buf));
}

void SystemMonitor::publishMemory(float memory)
{
    std::string value_str = std::to_string(memory);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    memory_pub->put(std::move(buf));
}

void SystemMonitor::publishTemperature(float temperature)
{
    std::string value_str = std::to_string(temperature);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    temperature_pub->put(std::move(buf));
}

void SystemMonitor::publishGpuUsage(float gpuUsage)
{
    std::string value_str = std::to_string(gpuUsage);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    gpuUsage_pub->put(std::move(buf));
}

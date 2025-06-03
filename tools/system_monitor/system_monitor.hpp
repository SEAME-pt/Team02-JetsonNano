#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>
#include <memory>

class SystemMonitor
{
  public:
    SystemMonitor(std::shared_ptr<zenoh::Session> session);

    void publishCpuLoad(float cpuLoad);
    void publishCpuUsage(float cpuUsage);
    void publishMemory(float memory);
    void publishTemperature(float temperature);
    void publishGpuUsage(float gpuUsage);

  private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;
    std::optional<zenoh::Publisher> cpuLoad_pub;
    std::optional<zenoh::Publisher> cpuUsage_pub;
    std::optional<zenoh::Publisher> memory_pub;
    std::optional<zenoh::Publisher> temperature_pub;
    std::optional<zenoh::Publisher> gpuUsage_pub;
};

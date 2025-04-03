#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>
#include <tuple>
#include <sys/sysinfo.h>
#include <cstdio>
#include <regex>
#include <string>

#include <zenoh.hxx>
#include "system_monitor.hpp"

/*
this should be runing with the zenoh router runing (with the prpoper config
file) - monitor_config.yml and the zenoh-router should be runing with the
updated config file - zenoh.json5 we should look int tegrastats results in the
car terminal to compare them with the ones we get from this program we should be
adding more of them maybe.
*/

using namespace zenoh;

float getCpuLoad()
{
    std::ifstream loadavg("/proc/loadavg");
    float load = 0.0f;
    if (loadavg >> load)
    {
        return load;
    }
    return 0.0f;
}

float getCpuUsage()
{
    auto readStats = []() -> std::tuple<long, long>
    {
        std::ifstream stat_file("/proc/stat");
        std::string line;
        std::getline(stat_file, line);
        std::istringstream iss(line);
        std::string cpu;
        long user, nice, system, idle, iowait, irq, softirq, steal;
        iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >>
            softirq >> steal;
        long total =
            user + nice + system + idle + iowait + irq + softirq + steal;
        long idleTime = idle + iowait;
        return std::make_tuple(total, idleTime);
    };

    long total1, idle1, total2, idle2;
    std::tie(total1, idle1) = readStats();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::tie(total2, idle2) = readStats();

    long totalDiff = total2 - total1;
    long idleDiff  = idle2 - idle1;
    if (totalDiff == 0)
        return 0.0f;
    float usage = 100.0f * (totalDiff - idleDiff) / totalDiff;
    return usage;
}

float getMemoryUsage()
{
    struct sysinfo info;
    if (sysinfo(&info) == 0)
    {
        long used          = info.totalram - info.freeram;
        float usagePercent = 100.0f * used / info.totalram;
        return usagePercent;
    }
    return 0.0f;
}

float getTemperature()
{
    std::ifstream temp_file("/sys/devices/virtual/thermal/thermal_zone0/temp");
    int temp;
    if (temp_file >> temp)
    {
        return temp / 1000.0f;
    }
    return 0.0f;
}

float getJetsonGpuUsage()
{
    // Run tegrastats for 2 seconds to capture a snapshot.
    FILE* pipe = popen("timeout 2 tegrastats", "r");
    if (!pipe)
    {
        std::cerr << "Failed to run tegrastats command" << std::endl;
        return -1.0f;
    }

    char buffer[256];
    std::string output;
    // Read all output from the pipe.
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        output += buffer;
    }
    pclose(pipe);

    // Adjust the regex to match the new GPU usage format: "GR3D_FREQ 16%"
    std::regex gpuRegex("GR3D_FREQ\\s+([0-9]+)%");
    std::smatch match;
    if (std::regex_search(output, match, gpuRegex))
    {
        int gpuUsage = std::stoi(match[1].str());
        return static_cast<float>(gpuUsage);
    }
    return -1.0f;
}

int main(int argc, char** argv)
{
    std::shared_ptr<zenoh::Session> session;

    if (argc == 2)
    {
        auto config = Config::from_file(argv[1]);
        session     = std::make_shared<zenoh::Session>(
            zenoh::Session::open(std::move(config)));
    }
    else
    {
        auto config = Config::create_default();
        session     = std::make_shared<zenoh::Session>(
            zenoh::Session::open(std::move(config)));
    }

    SystemMonitor monitor(session);

    while (true)
    {
        float cpuLoad     = getCpuLoad();
        float cpuUsage    = getCpuUsage();
        float memoryUsage = getMemoryUsage();
        float temperature = getTemperature();
        float gpuUsage    = getJetsonGpuUsage();

        // print

        std::cout << "Publishing stats: CPU Load: " << cpuLoad
                  << ", CPU Usage: " << cpuUsage
                  << ", Memory Usage: " << memoryUsage << ", GPU: " << gpuUsage
                  << ", Temperature: " << temperature << std::endl;

        // publish
        monitor.publishCpuLoad(cpuLoad);
        monitor.publishCpuUsage(cpuUsage);
        monitor.publishMemory(memoryUsage);
        monitor.publishTemperature(temperature);
        monitor.publishGpuUsage(gpuUsage);

        // delay seconds bfr looping again
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    return 0;
}

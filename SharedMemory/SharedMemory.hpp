#pragma once

#include <cstdint>
#include <mutex>

struct SharedMemory
{
    int32_t speed;
    int32_t direction;
    std::mutex mtx;
};

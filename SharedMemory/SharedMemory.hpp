#pragma once

#include <cstdint>
#include <pthread.h>

struct SharedMemory
{
    int32_t speed;
    int32_t direction;
    pthread_mutex_t mtx;
};

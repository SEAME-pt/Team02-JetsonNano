#pragma once

#include "PCA9685.hpp"
#include <algorithm>

class Motor
{
  public:
    enum Side
    {
        RIGHT,
        LEFT
    };

  private:
    PCA9685* m_MotorPCA;
    Side _side;
    int16_t m_currThrottle;

  public:
    Motor();
    ~Motor();

    void init(PCA9685* motorPCA, Side side);
    void setThrottle(int throttle);
};
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
    Side m_side;
    int16_t m_currThrottle;

  public:
    Motor();
    ~Motor();

    PCA9685* getMotorPCA(void) const;
    Side getSide(void) const;
    int16_t getThrottle(void) const;

    void setThrottle(int throttle);

    void init(PCA9685* motorPCA, Side side);
};
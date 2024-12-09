#pragma once

#include "PCA9685.hpp"
#include <algorithm>

enum Side
{
    RIGHT,
    LEFT
};

class Motor
{
  private:
    PCA9685* m_MotorPCA;
    Side _side;

  public:
    Motor();
    ~Motor();

    void init(PCA9685* motorPCA, Side side);
    void setSpeed(int angle);
};
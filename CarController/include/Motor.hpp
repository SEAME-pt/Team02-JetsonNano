#pragma once

#include "PCA9685.hpp"

class Motor
{
  private:
    PCA9685* m_MotorPCA;

  public:
    Motor();
    ~Motor();

    void init(PCA9685* motorPCA);
    void setSpeed(int angle);
};
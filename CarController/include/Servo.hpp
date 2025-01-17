#pragma once

#include "PCA9685.hpp"
#include <algorithm>

class Servo
{
  private:
    PCA9685* m_ServoPCA;
    uint16_t m_currDirection;

  public:
    Servo();
    ~Servo();

    void init(PCA9685* servoPCA);
    void setDirection(uint8_t angle);
};
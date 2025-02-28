#pragma once

#include "PCA9685.hpp"
#include "I2C.hpp"
#include <algorithm>
#include <memory>
#include <cstdint>

class ServoController
{
  private:
    std::unique_ptr<PCA9685> m_ServoPCA;
    int16_t m_currAngle;

  public:
    explicit ServoController(std::shared_ptr<I2C> i2c);
    ~ServoController();

    int16_t getAngle(void) const;

    void setAngle(int16_t angle);
};
#pragma once

#include "PCA9685.hpp"
#include "I2C.hpp"
#include <thread>
#include <algorithm>
#include <memory>
#include <cstdint>

class MotorController
{
  private:
    std::unique_ptr<PCA9685> m_MotorPCA;
    int32_t m_currSpeed;

  public:
    explicit MotorController(std::shared_ptr<I2C> i2c);
    ~MotorController();

    int32_t getSpeed(void) const;

    void setSpeed(int32_t speed);
};
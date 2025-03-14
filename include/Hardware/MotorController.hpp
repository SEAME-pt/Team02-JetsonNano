#pragma once

#include "PCA9685.hpp"
#include "I2C.hpp"
#include <thread>
#include <algorithm>
#include <memory>
#include <cstdint>

/**
 * @brief Motor controller via PCA9685
 *
 * @details Controls drive motor speed and direction:
 *          - Speed control (-100 to +100)
 *          - PWM signal generation
 *          - Direction control
 *          - Speed feedback
 *
 * Hardware specifications:
 * - Operating voltage: 12V
 * - PWM frequency: 1000Hz
 * - Speed range: Bidirectional
 *
 * Control values:
 * - Positive: Forward motion
 * - Negative: Reverse motion
 * - Zero: Stop
 *
 * @note Uses PCA9685 PWM controller via I2C
 * @see PCA9685
 * @see I2C
 */
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
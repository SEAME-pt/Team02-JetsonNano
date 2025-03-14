#pragma once

#include "PCA9685.hpp"
#include "I2C.hpp"
#include <algorithm>
#include <memory>
#include <cstdint>

/**
 * @brief Servo motor controller via PCA9685
 *
 * @details Controls steering servo motor position:
 *          - Angle control (0° to 180°)
 *          - PWM signal generation
 *          - Position feedback
 *
 * Hardware specifications:
 * - Operating voltage: 5V
 * - PWM frequency: 50Hz
 * - Angular range: 180°
 *
 * @note Uses PCA9685 PWM controller via I2C
 * @see PCA9685
 * @see I2C
 */
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
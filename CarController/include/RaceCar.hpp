#pragma once

#include "Motor.hpp"
#include "Servo.hpp"
#include "PCA9685.hpp"

class RaceCar
{
  private:
    I2C* m_I2c;
    PCA9685* m_ServoPCA;
    PCA9685* m_motorPCA;
    Motor motorRight;
    Motor motorLeft;
    Servo servo;

  public:
    RaceCar();
    ~RaceCar();

    void init(const std::string& i2cDevice, uint8_t motorAddress,
              uint8_t servoAddress);
    void setDirection(uint8_t angle);
    void setSpeed(int speed);
};
#pragma once

#include "PCA9685.hpp"

class RaceCar
{
  private:
    PCA9685* m_Motor;
    PCA9685* m_Servo;
    I2C* m_I2c;

  public:
    RaceCar();
    ~RaceCar();
    RaceCar(const RaceCar& originalRaceCar);
    RaceCar& operator=(const RaceCar& originalRaceCar);

    void init(const std::string& i2cDevice, uint8_t motorAddress,
              uint8_t servoAddress);
    void setDirection(uint8_t angle);
    void setSpeed(int speed);
};
#pragma once

#include <iostream>
#include "../include/I2C.hpp"

#define MODE1 0x00
#define MODE2 0x01
#define PRE_SCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

class PCA9685
{
  private:
    const int _deviceAddress;
    const I2C* m_i2c;

  public:
    PCA9685(const I2C* m_i2c, int deviceAddress);
    ~PCA9685();
    PCA9685(const PCA9685& originalPCA9685);
    PCA9685& operator=(const PCA9685& originalPCA9685);

    void setPWMFreq(float freq_hz);
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    void setDutyCicle(uint8_t channel, uint16_t pulseWidth);
    void setGPIO(uint8_t channel, uint16_t on);
};

#pragma once

#include <iostream>
#include <cstdint>
#include "../../I2C/include/I2C.hpp"

#define MODE1 0x00
#define PRE_SCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD

class PCA9685
{
  private:
    int _deviceAddress;
    I2C* _i2c;

  public:
    PCA9685();
    ~PCA9685();
    PCA9685(const PCA9685& originalPCA9685);
    PCA9685& operator=(const PCA9685& originalPCA9685);

    void init(I2C* m_i2c, int deviceAddress);
    void setPWMFreq(float freq_hz);
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    void setDutyCicle(uint8_t channel, uint16_t pulseWidth);
    void setGPIO(uint8_t channel, uint16_t on);
};

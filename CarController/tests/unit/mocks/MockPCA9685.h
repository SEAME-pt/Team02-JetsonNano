#pragma once
#include <gmock/gmock.h>
#include "PCA9685.hpp"

class MockPCA9685 : public PCA9685
{
  public:
    MOCK_METHOD(void, init, (I2C*, uint8_t), (override));
    MOCK_METHOD(void, setPWMFreq, (float), (override));
    MOCK_METHOD(void, setPWM, (uint8_t, uint16_t, uint16_t), (override));
    MOCK_METHOD(void, setDutyCicle, (uint8_t, uint16_t), (override));
    MOCK_METHOD(void, setGPIO, (uint8_t, uint16_t), (override));
};
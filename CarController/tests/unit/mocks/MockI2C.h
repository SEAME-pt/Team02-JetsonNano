#pragma once
#include <gmock/gmock.h>
#include "I2C.hpp"

class MockI2C : public I2C {
public:
    MOCK_METHOD(void, init, (const std::string&), (override));
    MOCK_METHOD(void, writeByte, (uint8_t, uint8_t, uint8_t), (override));
    MOCK_METHOD(uint8_t, readByte, (uint8_t), (override));
    MOCK_METHOD(void, writeMessage, (uint8_t, uint8_t*), (override));
    MOCK_METHOD(void, readRegister, (uint8_t, uint8_t, uint8_t*), (override));
    MOCK_METHOD(int, getFd, (), (override));
};
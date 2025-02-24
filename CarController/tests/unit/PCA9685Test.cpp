#include <gtest/gtest.h>
#include "PCA9685.hpp"
#include "mocks/MockI2C.h"

using ::testing::_;
using ::testing::Return;

class PCA9685Test : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        mockI2C = new MockI2C();
        pca     = new PCA9685();
    }

    void TearDown() override
    {
        delete pca;
        delete mockI2C;
    }

    MockI2C* mockI2C;
    PCA9685* pca;
};

TEST_F(PCA9685Test, InitializationTest)
{
    EXPECT_CALL(*mockI2C, writeByte(0x40, MODE1, 0x00)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, ALL_LED_ON_L, 0x00)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, ALL_LED_ON_H, 0x00)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, ALL_LED_OFF_L, 0x00)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, ALL_LED_OFF_H, 0x10)).Times(1);

    pca->init(mockI2C, 0x40);
}

TEST_F(PCA9685Test, SetPWMFreqTest)
{
    pca->init(mockI2C, 0x40);

    EXPECT_CALL(*mockI2C, writeByte(0x40, MODE1, 0x10)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, PRE_SCALE, _)).Times(1);

    pca->setPWMFreq(50);
}

TEST_F(PCA9685Test, SetPWMTest)
{
    pca->init(mockI2C, 0x40);

    EXPECT_CALL(*mockI2C, writeByte(0x40, LED0_ON_L + 4 * 0, _)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, LED0_ON_H + 4 * 0, _)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, LED0_OFF_L + 4 * 0, _)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, LED0_OFF_H + 4 * 0, _)).Times(1);

    pca->setPWM(0, 0, 2048);
}

TEST_F(PCA9685Test, DestructorTest)
{
    pca->init(mockI2C, 0x40);

    EXPECT_CALL(*mockI2C, writeByte(0x40, MODE1, 0x00)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, ALL_LED_ON_L, 0x00)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, ALL_LED_ON_H, 0x00)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, ALL_LED_OFF_L, 0x00)).Times(1);
    EXPECT_CALL(*mockI2C, writeByte(0x40, ALL_LED_OFF_H, 0x10)).Times(1);

    delete pca;
    pca = nullptr;
}
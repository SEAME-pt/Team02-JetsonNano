#include <gtest/gtest.h>
#include "RaceCar.hpp"
#include "mocks/MockI2C.h"
#include "mocks/MockPCA9685.h"

class RaceCarTest : public ::testing::Test {
protected:
    void SetUp() override {
        car = new RaceCar();
    }

    void TearDown() override {
        delete car;
    }

    RaceCar* car;
};

TEST_F(RaceCarTest, SubscriptionTest) {
    EXPECT_NO_THROW(car->init("/dev/i2c-1", 0x40, 0x41));
    
    // Test throttle subscription
    Sample throttleSample("50");
    car->m_subThrottle.callback(throttleSample);
    
    // Test direction subscription
    Sample directionSample("90");
    car->m_subDirection.callback(directionSample);
}

TEST_F(RaceCarTest, ComponentInitializationTest) {
    EXPECT_NO_THROW(car->init("/dev/i2c-1", 0x40, 0x41));
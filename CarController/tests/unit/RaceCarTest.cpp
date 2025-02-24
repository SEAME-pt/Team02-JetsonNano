#include <gtest/gtest.h>
#include "RaceCar.hpp"
#include "mocks/MockI2C.h"
#include "mocks/MockPCA9685.h"
#include <thread>
#include <chrono>

using namespace zenoh;

class RaceCarTest : public ::testing::Test
{
  protected:
    RaceCarTest()
        : car(new RaceCar()),
          testSession(Session::open(std::move(Config::create_default()))),
          pubThrottle(testSession.declare_publisher("seame/car/1/throttle")),
          pubDirection(testSession.declare_publisher("seame/car/1/direction"))
    {
    }
    void SetUp() override {}

    void TearDown() override
    {
        delete car;
        testSession.close();
    }

    RaceCar* car;
    Session testSession;
    Publisher pubThrottle;
    Publisher pubDirection;
};

TEST_F(RaceCarTest, ZenohPublishTest)
{
    EXPECT_NO_THROW(car->init("/dev/i2c-1", 0x40, 0x41));

    // Publish test throttle message
    pubThrottle.put("50");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_(car->getMotor)
    // Publish test direction message
    pubDirection.put("90");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
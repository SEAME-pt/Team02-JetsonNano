#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "VSSSubscriber.hpp"
#include "ControllerPublisher.hpp"
#include "Vehicle.hpp"
#include <thread>
#include <chrono>

TEST_CASE("Communication Integration Tests", "[communication]")
{
    auto config = zenoh::Config::create_default();

    std::shared_ptr<zenoh::Session> session = std::make_shared<zenoh::Session>(
        zenoh::Session::open(std::move(config)));
    REQUIRE(session != nullptr);

    Vehicle vehicle;
    bool canMessageReceived = false;
    uint32_t lastCanId      = 0;
    uint8_t lastCanData[8]  = {0};
    size_t lastCanLength    = 0;

    auto sendToCAN = [&](uint32_t id, uint8_t* data, size_t length)
    {
        canMessageReceived = true;
        lastCanId          = id;
        std::memcpy(lastCanData, data, length);
        lastCanLength = length;
    };

    VSSSubscriber subscriber(vehicle, sendToCAN, session);
    ControllerPublisher publisher(session);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    SECTION("Speed Control Integration")
    {
        const float testSpeed = 50.5f;
        publisher.publishSpeed(testSpeed);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        REQUIRE_THAT(vehicle.get_powertrain().get_electric_motor().get_speed(),
                     Catch::Matchers::WithinRel(testSpeed, 0.001f));
    }

    SECTION("Steering Control Integration")
    {
        const float testAngle = 30.0f;
        publisher.publishSteering(testAngle);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        REQUIRE_THAT(vehicle.get_chassis().get_steering_wheel().get_angle(),
                     Catch::Matchers::WithinRel(testAngle, 0.001f));
    }

    SECTION("Lights Control Integration")
    {
        SECTION("Beam Lights")
        {
            publisher.publishBeamLow(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(
                vehicle.get_body().get_lights().get_beam_low().get_is_on() ==
                true);
            REQUIRE(canMessageReceived == true);
            REQUIRE(lastCanId == 0x03);
            REQUIRE((lastCanData[0] & (1 << 2)) != 0);
        }

        SECTION("Direction Indicators")
        {
            publisher.publishDirectionIndicatorLeft(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(vehicle.get_body()
                        .get_lights()
                        .get_direction_indicator_left()
                        .get_is_signaling() == true);
            REQUIRE(canMessageReceived == true);
            REQUIRE(lastCanId == 0x03);
            REQUIRE((lastCanData[0] & (1 << 1)) != 0);
        }
    }

    SECTION("Transmission Control Integration")
    {
        const int testGear = 3;
        publisher.publishCurrentGear(testGear);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        REQUIRE(
            vehicle.get_powertrain().get_transmission().get_current_gear() ==
            testGear);
        REQUIRE(canMessageReceived == true);
        REQUIRE(lastCanId == 0x04);
        REQUIRE(lastCanData[0] == testGear);
    }
}
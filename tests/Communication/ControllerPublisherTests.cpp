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
        int32_t testSpeed = 50;
        publisher.publishSpeed(testSpeed);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        REQUIRE(vehicle.get_powertrain().get_electric_motor().get_speed() ==
                testSpeed);
    }

    SECTION("Steering Control Integration")
    {
        int16_t testAngle = 30;
        publisher.publishSteering(testAngle);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        REQUIRE(vehicle.get_chassis().get_steering_wheel().get_angle() ==
                testAngle);
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

            publisher.publishBeamHigh(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(
                vehicle.get_body().get_lights().get_beam_high().get_is_on() ==
                true);
            REQUIRE(canMessageReceived == true);
            REQUIRE(lastCanId == 0x03);
            REQUIRE((lastCanData[0] & (1 << 3)) != 0);
        }

        SECTION("Fog Lights")
        {
            publisher.publishFogRear(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(
                vehicle.get_body().get_lights().get_fog_rear().get_is_on() ==
                true);
            REQUIRE(canMessageReceived == true);
            REQUIRE(lastCanId == 0x03);
            REQUIRE((lastCanData[0] & (1 << 5)) != 0);

            publisher.publishFogFront(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(
                vehicle.get_body().get_lights().get_fog_front().get_is_on() ==
                true);
            REQUIRE(canMessageReceived == true);
            REQUIRE(lastCanId == 0x03);
            REQUIRE((lastCanData[0] & (1 << 4)) != 0);
        }

        SECTION("Brake Lights")
        {
            publisher.publishBrake(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(
                vehicle.get_body().get_lights().get_brake().get_is_active() ==
                true);
        }

        SECTION("Other Lights")
        {
            publisher.publishRunning(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(vehicle.get_body().get_lights().get_running().get_is_on() ==
                    true);

            publisher.publishParking(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(vehicle.get_body().get_lights().get_parking().get_is_on() ==
                    true);
            REQUIRE(canMessageReceived == true);
            REQUIRE(lastCanId == 0x03);
            REQUIRE((lastCanData[0] & (1 << 7)) != 0);

            publisher.publishHazard(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(vehicle.get_body()
                        .get_lights()
                        .get_hazard()
                        .get_is_signaling() == true);
            REQUIRE(canMessageReceived == true);
            REQUIRE(lastCanId == 0x03);
            REQUIRE((lastCanData[0] & (1 << 6)) != 0);
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

            publisher.publishDirectionIndicatorRight(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE(vehicle.get_body()
                        .get_lights()
                        .get_direction_indicator_right()
                        .get_is_signaling() == true);
            REQUIRE(canMessageReceived == true);
            REQUIRE(lastCanId == 0x03);
            REQUIRE((lastCanData[0] & (1 << 0)) != 0);
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
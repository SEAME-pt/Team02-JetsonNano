#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "VSSSubscriber.hpp"
#include "SensoringPublisher.hpp"
#include "Vehicle.hpp"
#include <thread>
#include <chrono>

TEST_CASE("Sensoring Integration Tests", "[sensoring]")
{
    auto config = zenoh::Config::create_default();

    std::shared_ptr<zenoh::Session> session = std::make_shared<zenoh::Session>(
        zenoh::Session::open(std::move(config)));
    REQUIRE(session != nullptr);

    Vehicle vehicle;

    VSSSubscriber subscriber(vehicle, session);
    SensoringPublisher publisher(session);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    SECTION("Vehicle Speed Publishing")
    {
        float testSpeed = 75.5f;
        publisher.publishSpeed(testSpeed);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        REQUIRE_THAT(vehicle.get_speed(),
                     Catch::Matchers::WithinRel(testSpeed, 0.001f));
    }

    SECTION("Battery Status Integration")
    {
        SECTION("Current Voltage")
        {
            float testVoltage = 400.0f;
            publisher.publishCurrentVoltage(testVoltage);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE_THAT(vehicle.get_powertrain()
                             .get_traction_battery()
                             .get_current_voltage(),
                         Catch::Matchers::WithinRel(testVoltage, 0.001f));
        }

        SECTION("Current Current")
        {
            float testCurrent = 150.0f;
            publisher.publishCurrentCurrent(testCurrent);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE_THAT(vehicle.get_powertrain()
                             .get_traction_battery()
                             .get_current_current(),
                         Catch::Matchers::WithinRel(testCurrent, 0.001f));
        }

        SECTION("Current Power")
        {
            float testPower = 60000.0f;
            publisher.publishCurrentPower(testPower);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE_THAT(vehicle.get_powertrain()
                             .get_traction_battery()
                             .get_current_power(),
                         Catch::Matchers::WithinRel(testPower, 0.001f));
        }

        SECTION("State of Charge")
        {
            float testSOC = 85.5f;
            publisher.publishStateOfCharge(testSOC);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            REQUIRE_THAT(vehicle.get_powertrain()
                             .get_traction_battery()
                             .get_state_of_charge_displayed(),
                         Catch::Matchers::WithinRel(testSOC, 0.001f));
        }
    }
}
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "TractionBattery.hpp"

TEST_CASE("TractionBattery Tests", "[traction_battery]")
{
    TractionBattery battery;

    SECTION("State of Charge Tests")
    {
        battery.set_state_of_charge_displayed(75.5f);
        REQUIRE_THAT(battery.get_state_of_charge_displayed(),
                     Catch::Matchers::WithinRel(75.5f, 0.001f));
        REQUIRE_THAT(battery.get_mutable_state_of_charge_displayed(),
                     Catch::Matchers::WithinRel(75.5f, 0.001f));
    }

    SECTION("Voltage Tests")
    {
        battery.set_max_voltage(400);
        REQUIRE(battery.get_max_voltage() == 400);

        battery.set_current_voltage(380.5f);
        REQUIRE_THAT(battery.get_current_voltage(),
                     Catch::Matchers::WithinRel(380.5f, 0.001f));
        REQUIRE_THAT(battery.get_mutable_current_voltage(),
                     Catch::Matchers::WithinRel(380.5f, 0.001f));
    }

    SECTION("Current Tests")
    {
        battery.set_current_current(50.2f);
        REQUIRE_THAT(battery.get_current_current(),
                     Catch::Matchers::WithinRel(50.2f, 0.001f));
        REQUIRE_THAT(battery.get_mutable_current_current(),
                     Catch::Matchers::WithinRel(50.2f, 0.001f));
    }

    SECTION("Power Tests")
    {
        battery.set_current_power(19000.5f);
        REQUIRE_THAT(battery.get_current_power(),
                     Catch::Matchers::WithinRel(19000.5f, 0.001f));
        REQUIRE_THAT(battery.get_mutable_current_power(),
                     Catch::Matchers::WithinRel(19000.5f, 0.001f));
    }
}
#include <catch2/catch_test_macros.hpp>
#include "TractionBattery.hpp"

TEST_CASE("TractionBattery Tests", "[traction_battery]")
{
    TractionBattery battery;

    SECTION("State of Charge Tests")
    {
        battery.set_state_of_charge_displayed(75.5f);
        REQUIRE(battery.get_state_of_charge_displayed() == Approx(75.5f));
        REQUIRE(battery.get_mutable_state_of_charge_displayed() ==
                Approx(75.5f));
    }

    SECTION("Voltage Tests")
    {
        battery.set_max_voltage(400);
        REQUIRE(battery.get_max_voltage() == 400);

        battery.set_current_voltage(380.5f);
        REQUIRE(battery.get_current_voltage() == Approx(380.5f));
        REQUIRE(battery.get_mutable_current_voltage() == Approx(380.5f));
    }

    SECTION("Current Tests")
    {
        battery.set_current_current(50.2f);
        REQUIRE(battery.get_current_current() == Approx(50.2f));
        REQUIRE(battery.get_mutable_current_current() == Approx(50.2f));
    }

    SECTION("Power Tests")
    {
        battery.set_current_power(19000.5f);
        REQUIRE(battery.get_current_power() == Approx(19000.5f));
        REQUIRE(battery.get_mutable_current_power() == Approx(19000.5f));
    }
}
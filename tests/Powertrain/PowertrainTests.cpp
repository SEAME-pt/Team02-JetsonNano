#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "Powertrain.hpp"

TEST_CASE("Powertrain Integration Tests", "[powertrain]")
{
    Powertrain powertrain;

    SECTION("Electric Motor Integration")
    {
        ElectricMotor motor;
        motor.set_max_power(300);
        motor.set_speed(120);

        powertrain.set_electric_motor(motor);
        REQUIRE(powertrain.get_electric_motor().get_max_power() == 300);
        REQUIRE(powertrain.get_electric_motor().get_speed() == 120);

        auto& mutable_motor = powertrain.get_mutable_electric_motor();
        mutable_motor.set_speed(150);
        REQUIRE(powertrain.get_electric_motor().get_speed() == 150);
    }

    SECTION("Range Tests")
    {
        powertrain.set_range(350);
        REQUIRE(powertrain.get_range() == 350);
    }

    SECTION("Time Remaining Tests")
    {
        powertrain.set_time_remaining(120);
        REQUIRE(powertrain.get_time_remaining() == 120);
    }

    SECTION("Transmission Integration")
    {
        Transmission trans;
        trans.set_current_gear(3);
        trans.set_drive_type("AWD");

        powertrain.set_transmission(trans);
        REQUIRE(powertrain.get_transmission().get_current_gear() == 3);
        REQUIRE(powertrain.get_transmission().get_drive_type() == "AWD");

        // Test mutable access
        auto& mutable_trans = powertrain.get_mutable_transmission();
        mutable_trans.set_current_gear(4);
        mutable_trans.set_drive_type("FWD");
        REQUIRE(powertrain.get_transmission().get_current_gear() == 4);
        REQUIRE(powertrain.get_transmission().get_drive_type() == "FWD");
    }

    SECTION("Traction Battery Integration")
    {
        TractionBattery battery;
        battery.set_state_of_charge_displayed(85.5f);
        battery.set_current_power(15000.0f);

        powertrain.set_traction_battery(battery);
        REQUIRE_THAT(
            powertrain.get_traction_battery().get_state_of_charge_displayed(),
            Catch::Matchers::WithinRel(85.5f, 0.001f));
        REQUIRE_THAT(powertrain.get_traction_battery().get_current_power(),
                     Catch::Matchers::WithinRel(15000.0f, 0.001f));

        // Test mutable access
        auto& mutable_battery = powertrain.get_mutable_traction_battery();
        mutable_battery.set_state_of_charge_displayed(90.0f);
        mutable_battery.set_current_power(16000.0f);
        REQUIRE_THAT(
            powertrain.get_traction_battery().get_state_of_charge_displayed(),
            Catch::Matchers::WithinRel(90.0f, 0.001f));
        REQUIRE_THAT(powertrain.get_traction_battery().get_current_power(),
                     Catch::Matchers::WithinRel(16000.0f, 0.001f));
    }
}
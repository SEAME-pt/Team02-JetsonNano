#include <catch2/catch_test_macros.hpp>
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
    }

    SECTION("Traction Battery Integration")
    {
        TractionBattery battery;
        battery.set_state_of_charge_displayed(85.5f);
        battery.set_current_power(15000.0f);

        powertrain.set_traction_battery(battery);
        REQUIRE(
            powertrain.get_traction_battery().get_state_of_charge_displayed() ==
            Approx(85.5f));
        REQUIRE(powertrain.get_traction_battery().get_current_power() ==
                Approx(15000.0f));
    }
}
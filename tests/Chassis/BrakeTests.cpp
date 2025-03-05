#include <catch2/catch_test_macros.hpp>
#include "Brake.hpp"

TEST_CASE("Brake Tests", "[brake]")
{
    Brake brake;

    SECTION("Emergency Braking Tests")
    {
        brake.set_is_driver_emergency_braking_detected(true);
        REQUIRE(brake.get_is_driver_emergency_braking_detected() == true);
    }

    SECTION("Pedal Position Tests")
    {
        brake.set_pedal_position(75);
        REQUIRE(brake.get_pedal_position() == 75);
    }
}
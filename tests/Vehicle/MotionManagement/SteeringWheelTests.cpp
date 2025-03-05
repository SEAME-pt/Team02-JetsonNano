#include <catch2/catch_test_macros.hpp>
#include "SteeringWheel.hpp"

TEST_CASE("SteeringWheel Tests", "[steering]")
{
    SteeringWheel wheel;

    SECTION("Angle Tests")
    {
        wheel.set_angle(45.5f);
        REQUIRE(wheel.get_angle() == Approx(45.5f));
    }

    SECTION("Target Angle Tests")
    {
        wheel.set_angle_target(30.0f);
        REQUIRE(wheel.get_angle_target() == Approx(30.0f));
    }

    SECTION("Angle Limits")
    {
        wheel.set_angle(-180.0f);
        REQUIRE(wheel.get_angle() == Approx(-180.0f));

        wheel.set_angle(180.0f);
        REQUIRE(wheel.get_angle() == Approx(180.0f));
    }
}
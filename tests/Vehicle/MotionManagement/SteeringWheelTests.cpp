#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "SteeringWheel.hpp"

TEST_CASE("SteeringWheel Tests", "[steering]")
{
    SteeringWheel wheel;

    SECTION("Angle Tests")
    {
        wheel.set_angle(45.5f);
        REQUIRE_THAT(wheel.get_angle(),
                     Catch::Matchers::WithinRel(45.5f, 0.001f));
    }

    SECTION("Target Angle Tests")
    {
        wheel.set_angle_target(30.0f);
        REQUIRE_THAT(wheel.get_angle_target(),
                     Catch::Matchers::WithinRel(30.0f, 0.001f));
    }

    SECTION("Angle Limits")
    {
        wheel.set_angle(-180.0f);
        REQUIRE_THAT(wheel.get_angle(),
                     Catch::Matchers::WithinRel(-180.0f, 0.001f));

        wheel.set_angle(180.0f);
        REQUIRE_THAT(wheel.get_angle(),
                     Catch::Matchers::WithinRel(180.0f, 0.001f));
    }
}
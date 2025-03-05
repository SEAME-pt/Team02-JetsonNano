#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "AngularVelocity.hpp"

TEST_CASE("AngularVelocity Tests", "[angular_velocity]")
{
    AngularVelocity angular_velocity;

    SECTION("Pitch Tests")
    {
        angular_velocity.set_pitch(15.5f);
        REQUIRE_THAT(angular_velocity.get_pitch(),
                     Catch::Matchers::WithinRel(15.5f, 0.001f));
    }

    SECTION("Roll Tests")
    {
        angular_velocity.set_roll(5.0f);
        REQUIRE_THAT(angular_velocity.get_roll(),
                     Catch::Matchers::WithinRel(5.0f, 0.001f));
    }

    SECTION("Yaw Tests")
    {
        angular_velocity.set_yaw(30.0f);
        REQUIRE_THAT(angular_velocity.get_yaw(),
                     Catch::Matchers::WithinRel(30.0f, 0.001f));
    }
}
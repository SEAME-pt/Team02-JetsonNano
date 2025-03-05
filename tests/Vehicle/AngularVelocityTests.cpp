#include <catch2/catch_test_macros.hpp>
#include "AngularVelocity.hpp"

TEST_CASE("AngularVelocity Tests", "[angular_velocity]")
{
    AngularVelocity angular_velocity;

    SECTION("Pitch Tests")
    {
        angular_velocity.set_pitch(15.5f);
        REQUIRE(angular_velocity.get_pitch() == Approx(15.5f));
    }

    SECTION("Roll Tests")
    {
        angular_velocity.set_roll(5.0f);
        REQUIRE(angular_velocity.get_roll() == Approx(5.0f));
    }

    SECTION("Yaw Tests")
    {
        angular_velocity.set_yaw(30.0f);
        REQUIRE(angular_velocity.get_yaw() == Approx(30.0f));
    }
}
#include <catch2/catch_test_macros.hpp>
#include "Steering.hpp"

TEST_CASE("Steering Tests", "[steering]")
{
    Steering steering;

    SECTION("Steering Wheel Integration")
    {
        SteeringWheel wheel;
        wheel.set_angle(15.5f);
        wheel.set_angle_target(20.0f);

        steering.set_steering_wheel(wheel);
        REQUIRE(steering.get_steering_wheel().get_angle() == Approx(15.5f));
        REQUIRE(steering.get_steering_wheel().get_angle_target() ==
                Approx(20.0f));

        auto& mutable_wheel = steering.get_mutable_steering_wheel();
        mutable_wheel.set_angle(25.0f);
        REQUIRE(steering.get_steering_wheel().get_angle() == Approx(25.0f));
    }
}
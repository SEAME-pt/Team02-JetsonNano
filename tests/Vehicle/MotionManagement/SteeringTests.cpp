#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
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
        REQUIRE_THAT(steering.get_steering_wheel().get_angle(),
                     Catch::Matchers::WithinRel(15.5f, 0.001f));
        REQUIRE_THAT(steering.get_steering_wheel().get_angle_target(),
                     Catch::Matchers::WithinRel(20.0f, 0.001f));

        auto& mutable_wheel = steering.get_mutable_steering_wheel();
        mutable_wheel.set_angle(25.0f);
        REQUIRE_THAT(steering.get_steering_wheel().get_angle(),
                     Catch::Matchers::WithinRel(25.0f, 0.001f));
    }
}
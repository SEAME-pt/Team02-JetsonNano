#include <catch2/catch_test_macros.hpp>
#include "Chassis.hpp"

TEST_CASE("Chassis Tests", "[chassis]")
{
    Chassis chassis;

    SECTION("Accelerator Tests")
    {
        Accelerator accelerator;
        accelerator.set_pedal_position(45);
        chassis.set_accelerator(accelerator);

        REQUIRE(chassis.get_accelerator().get_pedal_position() == 45);

        auto& mutable_accelerator = chassis.get_mutable_accelerator();
        mutable_accelerator.set_pedal_position(75);
        REQUIRE(chassis.get_accelerator().get_pedal_position() == 75);
    }

    SECTION("Axle Count Tests")
    {
        chassis.set_axle_count(2);
        REQUIRE(chassis.get_axle_count() == 2);
    }

    SECTION("Wheelbase Tests")
    {
        chassis.set_wheelbase(2700);
        REQUIRE(chassis.get_wheelbase() == 2700);
    }

    SECTION("Steering Wheel Tests")
    {
        ChassisSteeringWheel steering;
        steering.set_angle(30);
        chassis.set_steering_wheel(steering);

        REQUIRE(chassis.get_steering_wheel().get_angle() == 30);
    }
}
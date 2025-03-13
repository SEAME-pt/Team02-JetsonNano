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

    SECTION("Axle Tests")
    {
        Axle axle;
        auto& row1 = axle.get_mutable_row1();
        row1.set_track_width(1600);
        chassis.set_axle(axle);

        REQUIRE(chassis.get_axle().get_row1().get_track_width() == 1600);

        auto& mutable_axle = chassis.get_mutable_axle();
        auto& mutable_row  = mutable_axle.get_mutable_row1();
        mutable_row.set_track_width(1650);
        REQUIRE(chassis.get_axle().get_row1().get_track_width() == 1650);
    }

    SECTION("Brake Tests")
    {
        Brake brake;
        brake.set_pedal_position(30);
        chassis.set_brake(brake);

        REQUIRE(chassis.get_brake().get_pedal_position() == 30);

        auto& mutable_brake = chassis.get_mutable_brake();
        mutable_brake.set_pedal_position(45);
        REQUIRE(chassis.get_brake().get_pedal_position() == 45);
    }

    SECTION("Steering Wheel Tests")
    {
        ChassisSteeringWheel steering;
        steering.set_angle(30);
        chassis.set_steering_wheel(steering);

        REQUIRE(chassis.get_steering_wheel().get_angle() == 30);

        auto& mutable_steering = chassis.get_mutable_steering_wheel();
        mutable_steering.set_angle(45);
        REQUIRE(chassis.get_steering_wheel().get_angle() == 45);
    }
}
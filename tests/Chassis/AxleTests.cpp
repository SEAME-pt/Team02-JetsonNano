#include <catch2/catch_test_macros.hpp>
#include "Axle.hpp"

TEST_CASE("Wheel Tests", "[wheel]")
{
    Wheel wheel;

    SECTION("Angular Speed Tests")
    {
        wheel.set_angular_speed(45.5f);
        REQUIRE(wheel.get_angular_speed() == 45.5f);
    }

    SECTION("Speed Tests")
    {
        wheel.set_speed(120.0f);
        REQUIRE(wheel.get_speed() == 120.0f);
    }
}

TEST_CASE("AxleRow Tests", "[axle_row]")
{
    AxleRow axle_row;

    SECTION("Dimension Tests")
    {
        axle_row.set_axle_width(1800);
        REQUIRE(axle_row.get_axle_width() == 1800);

        axle_row.set_track_width(1600);
        REQUIRE(axle_row.get_track_width() == 1600);

        axle_row.set_tread_width(1400);
        REQUIRE(axle_row.get_tread_width() == 1400);

        axle_row.set_tire_width(205);
        REQUIRE(axle_row.get_tire_width() == 205);

        axle_row.set_wheel_width(8.5f);
        REQUIRE(axle_row.get_wheel_width() == 8.5f);

        axle_row.set_wheel_diameter(18.0f);
        REQUIRE(axle_row.get_wheel_diameter() == 18.0f);
    }

    SECTION("Tire Tests")
    {
        axle_row.set_tire_aspect_ratio(55);
        REQUIRE(axle_row.get_tire_aspect_ratio() == 55);

        axle_row.set_tire_diameter(17.0f);
        REQUIRE(axle_row.get_tire_diameter() == 17.0f);
    }

    SECTION("Wheel Count and Steering Tests")
    {
        axle_row.set_wheel_count(2);
        REQUIRE(axle_row.get_wheel_count() == 2);

        axle_row.set_steering_angle(15.5f);
        REQUIRE(axle_row.get_steering_angle() == 15.5f);
    }

    SECTION("Wheel Integration Tests")
    {
        // Test left wheel
        Wheel left_wheel;
        left_wheel.set_speed(100.0f);
        axle_row.set_wheel_left(left_wheel);
        REQUIRE(axle_row.get_wheel_left().get_speed() == 100.0f);

        // Test right wheel
        Wheel right_wheel;
        right_wheel.set_speed(95.0f);
        axle_row.set_wheel_right(right_wheel);
        REQUIRE(axle_row.get_wheel_right().get_speed() == 95.0f);

        // Test mutable access
        auto& mutable_left = axle_row.get_mutable_wheel_left();
        mutable_left.set_angular_speed(30.0f);
        REQUIRE(axle_row.get_wheel_left().get_angular_speed() == 30.0f);

        auto& mutable_right = axle_row.get_mutable_wheel_right();
        mutable_right.set_angular_speed(28.0f);
        REQUIRE(axle_row.get_wheel_right().get_angular_speed() == 28.0f);
    }
}

TEST_CASE("Axle Tests", "[axle]")
{
    Axle axle;

    SECTION("Row Tests")
    {
        AxleRow row1;
        row1.set_wheel_count(2);
        row1.set_track_width(1600);
        axle.set_row1(row1);
        REQUIRE(axle.get_row1().get_wheel_count() == 2);
        REQUIRE(axle.get_row1().get_track_width() == 1600);

        AxleRow row2;
        row2.set_wheel_count(2);
        row2.set_track_width(1650);
        axle.set_row2(row2);
        REQUIRE(axle.get_row2().get_wheel_count() == 2);
        REQUIRE(axle.get_row2().get_track_width() == 1650);

        // Test mutable access
        auto& mutable_row1 = axle.get_mutable_row1();
        mutable_row1.set_steering_angle(20.0f);
        REQUIRE(axle.get_row1().get_steering_angle() == 20.0f);

        auto& mutable_row2 = axle.get_mutable_row2();
        mutable_row2.set_steering_angle(0.0f);
        REQUIRE(axle.get_row2().get_steering_angle() == 0.0f);
    }
}
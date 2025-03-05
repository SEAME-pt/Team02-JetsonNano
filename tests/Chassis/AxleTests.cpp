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
    }

    SECTION("Tire Tests")
    {
        axle_row.set_tire_aspect_ratio(55);
        REQUIRE(axle_row.get_tire_aspect_ratio() == 55);

        axle_row.set_tire_diameter(17.0f);
        REQUIRE(axle_row.get_tire_diameter() == 17.0f);
    }

    SECTION("Wheel Integration Tests")
    {
        Wheel left_wheel;
        left_wheel.set_speed(100.0f);
        axle_row.set_wheel_left(left_wheel);

        REQUIRE(axle_row.get_wheel_left().get_speed() == 100.0f);
    }
}
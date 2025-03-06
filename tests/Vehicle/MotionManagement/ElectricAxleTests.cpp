#include <catch2/catch_test_macros.hpp>
#include "ElectricAxle.hpp"

TEST_CASE("ElectricAxleRow Tests", "[electric_axle]")
{
    ElectricAxleRow axle_row;

    SECTION("Rotational Speed Tests")
    {
        axle_row.set_rotational_speed(1500);
        REQUIRE(axle_row.get_rotational_speed() == 1500);
    }

    SECTION("Target Speed Tests")
    {
        axle_row.set_rotational_speed_target(2000);
        REQUIRE(axle_row.get_rotational_speed_target() == 2000);
    }
}

TEST_CASE("ElectricAxle Tests", "[electric_axle]")
{
    ElectricAxle axle;

    SECTION("Row1 Tests")
    {
        ElectricAxleRow row1;
        row1.set_rotational_speed(1800);
        row1.set_rotational_speed_target(2200);

        axle.set_row1(row1);
        REQUIRE(axle.get_row1().get_rotational_speed() == 1800);
        REQUIRE(axle.get_row1().get_rotational_speed_target() == 2200);

        auto& mutable_row1 = axle.get_mutable_row1();
        mutable_row1.set_rotational_speed(2000);
        REQUIRE(axle.get_row1().get_rotational_speed() == 2000);
    }

    SECTION("Row2 Tests")
    {
        ElectricAxleRow row2;
        row2.set_rotational_speed(1800);
        row2.set_rotational_speed_target(2200);

        axle.set_row2(row2);
        REQUIRE(axle.get_row2().get_rotational_speed() == 1800);
        REQUIRE(axle.get_row2().get_rotational_speed_target() == 2200);

        auto& mutable_row1 = axle.get_mutable_row2();
        mutable_row1.set_rotational_speed(2000);
        REQUIRE(axle.get_row2().get_rotational_speed() == 2000);
    }
}
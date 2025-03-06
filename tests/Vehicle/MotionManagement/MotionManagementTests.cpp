#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "MotionManagement.hpp"

TEST_CASE("MotionManagement Integration Tests", "[motion_management]")
{
    MotionManagement management;

    SECTION("Electric Axle Integration")
    {
        ElectricAxle axle;
        auto& row1 = axle.get_mutable_row1();
        row1.set_rotational_speed(1500);
        row1.set_rotational_speed_target(1800);

        management.set_electric_axle(axle);
        REQUIRE(
            management.get_electric_axle().get_row1().get_rotational_speed() ==
            1500);
        REQUIRE(management.get_electric_axle()
                    .get_row1()
                    .get_rotational_speed_target() == 1800);

        auto& mutable_axle = management.get_mutable_electric_axle();
        mutable_axle.get_mutable_row1().set_rotational_speed(2000);
        REQUIRE(
            management.get_electric_axle().get_row1().get_rotational_speed() ==
            2000);

        auto& mutable_axle = management.get_mutable_electric_axle();
        auto& row2         = mutable_axle.get_mutable_row2();
        row2.set_rotational_speed(1200);
        row2.set_rotational_speed_target(1500);

        REQUIRE(
            management.get_electric_axle().get_row2().get_rotational_speed() ==
            1200);
        REQUIRE(management.get_electric_axle()
                    .get_row2()
                    .get_rotational_speed_target() == 1500);
    }

    SECTION("Steering Integration")
    {
        Steering steering;
        auto& wheel = steering.get_mutable_steering_wheel();
        wheel.set_angle(30.0f);
        wheel.set_angle_target(45.0f);

        management.set_steering(steering);
        REQUIRE_THAT(management.get_steering().get_steering_wheel().get_angle(),
                     Catch::Matchers::WithinRel(30.0f, 0.001f));
        REQUIRE_THAT(
            management.get_steering().get_steering_wheel().get_angle_target(),
            Catch::Matchers::WithinRel(45.0f, 0.001f));
    }
}
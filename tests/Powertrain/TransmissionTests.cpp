#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "Transmission.hpp"

TEST_CASE("Transmission Tests", "[transmission]")
{
    Transmission transmission;

    SECTION("Current Gear Tests")
    {
        transmission.set_current_gear(-1);
        REQUIRE(transmission.get_current_gear() == -1);

        transmission.set_current_gear(1);
        REQUIRE(transmission.get_current_gear() == 1);
    }

    SECTION("Drive Type Tests")
    {
        transmission.set_drive_type("AWD");
        REQUIRE(transmission.get_drive_type() == "AWD");

        std::string& mutable_type = transmission.get_mutable_drive_type();
        mutable_type              = "FWD";
        REQUIRE(transmission.get_drive_type() == "FWD");
    }

    SECTION("Gear Change Mode Tests")
    {
        transmission.set_gear_change_mode("AUTOMATIC");
        REQUIRE(transmission.get_gear_change_mode() == "AUTOMATIC");
    }

    SECTION("Park Lock Tests")
    {
        transmission.set_is_park_lock_engaged(true);
        REQUIRE(transmission.get_is_park_lock_engaged() == true);
    }

    SECTION("Performance Mode Tests")
    {
        transmission.set_performance_mode("SPORT");
        REQUIRE(transmission.get_performance_mode() == "SPORT");
    }

    SECTION("Selected Gear Tests")
    {
        transmission.set_selected_gear(2);
        REQUIRE(transmission.get_selected_gear() == 2);
    }

    SECTION("Travelled Distance Tests")
    {
        transmission.set_travelled_distance(150.5f);
        REQUIRE_THAT(transmission.get_travelled_distance(),
                     Catch::Matchers::WithinRel(150.5f, 0.001f));
    }
}
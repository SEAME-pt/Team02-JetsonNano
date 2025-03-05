#include <catch2/catch_test_macros.hpp>
#include "ADAS.hpp"

TEST_CASE("ADAS Integration Tests", "[adas]")
{
    ADAS adas;

    SECTION("Autonomy Level Tests")
    {
        adas.set_active_autonomy_level("Level 2");
        adas.set_supported_autonomy_level("Level 3");
        REQUIRE(adas.get_active_autonomy_level() == "Level 2");
        REQUIRE(adas.get_supported_autonomy_level() == "Level 3");
    }

    SECTION("Obstacle Detection Integration")
    {
        ObstacleDetection detection;
        auto& front = detection.get_mutable_front();
        auto& rear  = detection.get_mutable_rear();

        front.set_distance(4.0f);
        front.set_is_warning(true);
        rear.set_distance(2.0f);
        rear.set_is_warning(true);

        adas.set_obstacle_detection(detection);

        const auto& result = adas.get_obstacle_detection();
        REQUIRE(result.get_front().get_distance() == 4.0f);
        REQUIRE(result.get_front().get_is_warning() == true);
        REQUIRE(result.get_rear().get_distance() == 2.0f);
        REQUIRE(result.get_rear().get_is_warning() == true);
    }
}
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
        auto& mutable_obstacle = adas.get_mutable_obstacle_detection();
        auto& front_mutable    = mutable_obstacle.get_mutable_front();
        auto& rear_mutable     = mutable_obstacle.get_mutable_rear();

        front_mutable.set_distance(4.0f);
        front_mutable.set_is_warning(true);
        rear_mutable.set_distance(2.0f);
        rear_mutable.set_is_warning(true);

        REQUIRE(mutable_obstacle.get_front().get_distance() == 4.0f);
        REQUIRE(mutable_obstacle.get_front().get_is_warning() == true);
        REQUIRE(mutable_obstacle.get_rear().get_distance() == 2.0f);
        REQUIRE(mutable_obstacle.get_rear().get_is_warning() == true);

        ObstacleDetection detection;

        auto& front = detection.get_mutable_front();
        auto& rear  = detection.get_mutable_rear();

        front.set_distance(4.0f);
        front.set_is_warning(true);
        rear.set_distance(2.0f);
        rear.set_is_warning(true);

        adas.set_obstacle_detection(detection);

        const auto& result2 = adas.get_obstacle_detection();
        REQUIRE(result2.get_front().get_distance() == 4.0f);
        REQUIRE(result2.get_front().get_is_warning() == true);
        REQUIRE(result2.get_rear().get_distance() == 2.0f);
        REQUIRE(result2.get_rear().get_is_warning() == true);
    }
}
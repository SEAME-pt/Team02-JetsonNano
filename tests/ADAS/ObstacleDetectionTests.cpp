#include <catch2/catch_test_macros.hpp>
#include "ObstacleDetection.hpp"

TEST_CASE("ObstacleDetection Tests", "[obstacle_detection]")
{
    ObstacleDetection obstacle_detection;

    SECTION("Front Detection Tests")
    {
        Detection front;
        front.set_distance(5.0f);
        front.set_is_enabled(true);
        front.set_warning_type("FRONT_ALERT");

        obstacle_detection.set_front(front);
        REQUIRE(obstacle_detection.get_front().get_distance() == 5.0f);
        REQUIRE(obstacle_detection.get_front().get_is_enabled() == true);
        REQUIRE(obstacle_detection.get_front().get_warning_type() ==
                "FRONT_ALERT");
    }

    SECTION("Rear Detection Tests")
    {
        Detection rear;
        rear.set_distance(3.0f);
        rear.set_is_enabled(true);
        rear.set_warning_type("REAR_ALERT");

        obstacle_detection.set_rear(rear);
        REQUIRE(obstacle_detection.get_rear().get_distance() == 3.0f);
        REQUIRE(obstacle_detection.get_rear().get_is_enabled() == true);
        REQUIRE(obstacle_detection.get_rear().get_warning_type() ==
                "REAR_ALERT");
    }

    SECTION("Mutable Access Tests")
    {
        auto& mutable_front = obstacle_detection.get_mutable_front();
        mutable_front.set_distance(7.5f);
        REQUIRE(obstacle_detection.get_front().get_distance() == 7.5f);

        auto& mutable_rear = obstacle_detection.get_mutable_rear();
        mutable_rear.set_distance(2.5f);
        REQUIRE(obstacle_detection.get_rear().get_distance() == 2.5f);
    }
}

TEST_CASE("Detection Tests", "[detection]")
{
    Detection detection;

    SECTION("Distance Tests")
    {
        detection.set_distance(10.5f);
        REQUIRE(detection.get_distance() == 10.5f);
    }

    SECTION("Enable/Disable Tests")
    {
        detection.set_is_enabled(true);
        REQUIRE(detection.get_is_enabled() == true);
        detection.set_is_enabled(false);
        REQUIRE(detection.get_is_enabled() == false);
    }

    SECTION("Error State Tests")
    {
        detection.set_is_error(true);
        REQUIRE(detection.get_is_error() == true);
    }

    SECTION("Warning State Tests")
    {
        detection.set_is_warning(true);
        REQUIRE(detection.get_is_warning() == true);
    }

    SECTION("Time Gap Tests")
    {
        detection.set_time_gap(500);
        REQUIRE(detection.get_time_gap() == 500);
    }

    SECTION("Warning Type Tests")
    {
        detection.set_warning_type("COLLISION_ALERT");
        REQUIRE(detection.get_warning_type() == "COLLISION_ALERT");

        auto& mutable_warning = detection.get_mutable_warning_type();
        mutable_warning       = "PROXIMITY_WARNING";
        REQUIRE(detection.get_warning_type() == "PROXIMITY_WARNING");
    }
}
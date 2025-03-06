#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "Vehicle.hpp"

TEST_CASE("Vehicle Tests", "[vehicle]")
{
    Vehicle vehicle;

    SECTION("Basic Properties Tests")
    {
        vehicle.set_average_speed(60.5f);
        REQUIRE_THAT(vehicle.get_average_speed(),
                     Catch::Matchers::WithinRel(60.5f, 0.001f));

        vehicle.set_current_overall_weight(1500);
        REQUIRE(vehicle.get_current_overall_weight() == 1500);

        vehicle.set_height(1450);
        REQUIRE(vehicle.get_height() == 1450);

        vehicle.set_length(4200);
        REQUIRE(vehicle.get_length() == 4200);

        vehicle.set_is_moving(true);
        REQUIRE(vehicle.get_is_moving() == true);
    }

    SECTION("Movement Tests")
    {
        vehicle.set_speed(100.5f);
        REQUIRE_THAT(vehicle.get_speed(),
                     Catch::Matchers::WithinRel(100.5f, 0.001f));

        vehicle.set_traveled_distance(150.75f);
        REQUIRE_THAT(vehicle.get_traveled_distance(),
                     Catch::Matchers::WithinRel(150.75f, 0.001f));

        vehicle.set_trip_duration(3600.0f);
        REQUIRE_THAT(vehicle.get_trip_duration(),
                     Catch::Matchers::WithinRel(3600.0f, 0.001f));

        vehicle.set_turning_diameter(11000);
        REQUIRE(vehicle.get_turning_diameter() == 11000);
    }

    SECTION("Component Integration Tests")
    {
        // Test ADAS
        ADAS adas;
        adas.set_active_autonomy_level("Level 2");
        vehicle.set_adas(adas);
        REQUIRE(vehicle.get_adas().get_active_autonomy_level() == "Level 2");

        // Test Acceleration
        Acceleration accel;
        accel.set_lateral(0.5f);
        vehicle.set_acceleration(accel);
        REQUIRE_THAT(vehicle.get_acceleration().get_lateral(),
                     Catch::Matchers::WithinRel(0.5f, 0.001f));

        // Test AngularVelocity
        AngularVelocity ang_vel;
        ang_vel.set_pitch(15.0f);
        vehicle.set_angular_velocity(ang_vel);
        REQUIRE_THAT(vehicle.get_angular_velocity().get_pitch(),
                     Catch::Matchers::WithinRel(15.0f, 0.001f));

        // Test Body
        Body body;
        vehicle.set_body(body);
        auto& mutable_body   = vehicle.get_mutable_body();
        auto& mutable_lights = mutable_body.get_mutable_lights();
        mutable_lights.set_running(
            StaticLights()); // Changed from set_is_moving
        REQUIRE(vehicle.get_body().get_lights().get_running().get_is_on() ==
                false);

        // Test Connectivity
        Connectivity conn;
        conn.set_connection_status(true); // Changed from set_is_connected
        vehicle.set_connectivity(conn);
        REQUIRE(vehicle.get_connectivity().get_connection_status() == true);

        // Test Exterior
        Exterior ext;
        vehicle.set_exterior(ext);
        auto& mutable_ext = vehicle.get_mutable_exterior();
        mutable_ext.set_trunk_angle(45.0f); // Changed from set_hood_angle
        REQUIRE_THAT(vehicle.get_exterior()
                         .get_trunk_angle(), // Changed from get_hood_angle
                     Catch::Matchers::WithinRel(45.0f, 0.001f));

        // Test MotionManagement
        MotionManagement motion;
        vehicle.set_motion_management(motion);
        auto& mutable_motion   = vehicle.get_mutable_motion_management();
        auto& mutable_steering = mutable_motion.get_mutable_steering();
        mutable_steering.get_mutable_steering_wheel().set_angle(30.0f);
        REQUIRE_THAT(vehicle.get_motion_management()
                         .get_steering()
                         .get_steering_wheel()
                         .get_angle(),
                     Catch::Matchers::WithinRel(30.0f, 0.001f));
    }

    SECTION("Time and Distance Tests")
    {
        vehicle.set_start_time("2024-03-05T15:30:00Z");
        REQUIRE(vehicle.get_start_time() == "2024-03-05T15:30:00Z");

        vehicle.set_traveled_distance_since_start(500.75f);
        REQUIRE_THAT(vehicle.get_traveled_distance_since_start(),
                     Catch::Matchers::WithinRel(500.75f, 0.001f));
    }

    SECTION("Mutable Component Access Tests")
    {
        auto& mutable_adas = vehicle.get_mutable_adas();
        mutable_adas.set_active_autonomy_level("Level 3");
        REQUIRE(vehicle.get_adas().get_active_autonomy_level() == "Level 3");

        auto& mutable_accel = vehicle.get_mutable_acceleration();
        mutable_accel.set_longitudinal(2.5f);
        REQUIRE_THAT(vehicle.get_acceleration().get_longitudinal(),
                     Catch::Matchers::WithinRel(2.5f, 0.001f));

        auto& mutable_ang_vel = vehicle.get_mutable_angular_velocity();
        mutable_ang_vel.set_roll(10.0f);
        REQUIRE_THAT(vehicle.get_angular_velocity().get_roll(),
                     Catch::Matchers::WithinRel(10.0f, 0.001f));

        auto& mutable_connectivity = vehicle.get_mutable_connectivity();
        mutable_connectivity.set_is_connected(false);
        REQUIRE(vehicle.get_connectivity().get_is_connected() == false);
    }
}
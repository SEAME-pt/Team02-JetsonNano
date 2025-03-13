#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "VehicleSystem.hpp"
#include <fstream>

TEST_CASE("VehicleSystem Tests", "[vehicle_system]")
{
    SECTION("Default Configuration")
    {
        auto mock_i2c = std::make_shared<I2C>();
        auto mock_can = std::make_shared<CAN>();

        VehicleSystem system;
        system.setI2C(mock_i2c);
        system.setCAN(mock_can);

        // Test initial vehicle state
        REQUIRE(system.getVehicle().get_is_moving() == false);
        REQUIRE(system.getVehicle().get_speed() == 0.0f);

        // Test controllers
        REQUIRE(system.getMotorController() != nullptr);
        REQUIRE(system.getServoController() != nullptr);
    }

    // SECTION("Custom Configuration File")
    // {
    //     const char* config_path = "test_config.json";
    //     {
    //         std::ofstream config_file(config_path);
    //         config_file << R"({
    //             "mode": "client",
    //             "connect": {
    //                 "endpoints": ["tcp/localhost:7447"]
    //             }
    //         })";
    //     }

    //     VehicleSystem system(config_path);
    //     REQUIRE(system.getSession() != nullptr);

    //     remove(config_path);
    // }

    SECTION("Communication Interface Tests")
    {
        VehicleSystem system;

        // Test VSS Subscriber
        const auto& subscriber = system.getVSSSubscriber();
        REQUIRE(subscriber != nullptr);

        // Test VSS Queryable
        const auto& queryable = system.getVSSQueryable();
        REQUIRE(queryable != nullptr);
    }

    // SECTION("Hardware Control Tests")
    // {
    //     VehicleSystem system;

    //     // Test Motor Control
    //     auto motor = system.getMotorController();
    //     REQUIRE(motor != nullptr);

    //     // Test speed control
    //     motor->setSpeed(50);
    //     REQUIRE(system.getVehicle().get_speed() == 50);

    //     // Test Servo Control
    //     auto servo = system.getServoController();
    //     REQUIRE(servo != nullptr);

    //     // Test steering control
    //     servo->setAngle(30.0f);
    //     REQUIRE_THAT(
    //         system.getVehicle().get_chassis().get_steering_wheel().get_angle(),
    //         Catch::Matchers::WithinRel(30.0f, 0.001f));
    // }

    SECTION("Error Handling Tests")
    {
        // Test invalid config file
        REQUIRE_THROWS_AS(VehicleSystem("invalid_config.json"),
                          std::runtime_error);
    }
}
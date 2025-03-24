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

        // Test component getters/setters
        REQUIRE(system.getI2C() == mock_i2c);
        REQUIRE(system.getCAN() == mock_can);

        // Test controllers initialization
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

    SECTION("Vehicle State Modification Tests")
    {
        VehicleSystem system;

        // Test vehicle getter/setter
        auto& mutable_vehicle = system.getMutableVehicle();
        mutable_vehicle.set_speed(50.0f);
        REQUIRE(system.getVehicle().get_speed() == 50.0f);

        REQUIRE(system.getSession() != nullptr);
    }

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

    // SECTION("Communication Components Tests")
    // {
    //     VehicleSystem system;

    //     // Test VSS Subscriber getter/setter
    //     auto test_subscriber = std::make_unique<VSSSubscriber>();
    //     system.setVSSSubscriber(std::move(test_subscriber));
    //     REQUIRE(system.getVSSSubscriber() != nullptr);

    //     // Test VSS Queryable getter/setter
    //     auto test_queryable = std::make_unique<VSSQueryable>();
    //     system.setVSSQueryable(std::move(test_queryable));
    //     REQUIRE(system.getVSSQueryable() != nullptr);

    //     // Test mutable access
    //     auto& mutable_subscriber = system.getMutableVSSSubscriber();
    //     REQUIRE(&mutable_subscriber == system.getVSSSubscriber().get());

    //     auto& mutable_queryable = system.getMutableVSSQueryable();
    //     REQUIRE(&mutable_queryable == system.getVSSQueryable().get());
    // }

    SECTION("Hardware Controller Tests")
    {
        VehicleSystem system;
        auto mock_i2c = std::make_shared<I2C>();

        // Test Motor Controller getter/setter
        auto test_motor = std::make_shared<MotorController>(mock_i2c);
        system.setMotorController(test_motor);
        REQUIRE(system.getMotorController() == test_motor);

        // Test Servo Controller getter/setter
        auto test_servo = std::make_shared<ServoController>(mock_i2c);
        system.setServoController(test_servo);
        REQUIRE(system.getServoController() == test_servo);
    }

    SECTION("Error Handling Tests")
    {
        // Test invalid config file
        REQUIRE_THROWS_AS(VehicleSystem("invalid_config.json"),
                          std::runtime_error);
    }
}
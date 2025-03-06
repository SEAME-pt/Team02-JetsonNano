#include <catch2/catch_test_macros.hpp>
#include "VehicleSystem.hpp"
#include <filesystem>

class MockI2C : public I2C
{
  public:
    void init(const std::string&) override {}
    void writeByte(uint8_t, uint8_t, uint8_t) override {}
    uint8_t readByte(uint8_t) override { return 0; }
    void readRegister(uint8_t, uint8_t, uint8_t*) override {}
};

class MockCAN : public CAN
{
  public:
    void init(const std::string&) override {}
    void writeMessage(uint32_t, uint8_t*, size_t) override {}
    uint8_t readMessage(uint8_t, uint32_t&, uint8_t*) override { return 0; }
};

TEST_CASE("VehicleSystem Tests", "[vehicle_system]")
{
    SECTION("Default Configuration")
    {
        auto mock_i2c = std::make_shared<MockI2C>();
        auto mock_can = std::make_shared<MockCAN>();

        VehicleSystem system;

        // Test initial vehicle state
        REQUIRE(system.getVehicle().get_is_moving() == false);
        REQUIRE(system.getVehicle().get_speed() == 0.0f);

        // Test controllers initialization
        REQUIRE(system.getMotorController() != nullptr);
        REQUIRE(system.getServoController() != nullptr);
    }

    SECTION("Custom Configuration File")
    {
        std::string config_path = "test_config.json";
        std::ofstream config_file(config_path);
        config_file << R"({
            "mode": "client",
            "connect": {
                "endpoints": ["tcp/localhost:7447"]
            }
        })";
        config_file.close();

        VehicleSystem system(config_path);
        REQUIRE(system.getSession() != nullptr);

        std::filesystem::remove(config_path);
    }

    SECTION("Communication Interface Tests")
    {
        VehicleSystem system;

        // Test VSS Subscriber
        auto subscriber = system.getVSSSubscriber();
        REQUIRE(subscriber != nullptr);
        REQUIRE(subscriber->isConnected() == true);

        // Test VSS Queryable
        auto queryable = system.getVSSQueryable();
        REQUIRE(queryable != nullptr);
        REQUIRE(queryable->isConnected() == true);
    }

    SECTION("Hardware Control Tests")
    {
        VehicleSystem system;

        // Test Motor Control
        auto motor = system.getMotorController();
        REQUIRE(motor != nullptr);
        REQUIRE(motor->isInitialized() == true);

        // Test speed control
        motor->setSpeed(50.0f);
        REQUIRE_THAT(system.getVehicle().get_speed(),
                     Catch::Matchers::WithinRel(50.0f, 0.001f));

        // Test Servo Control
        auto servo = system.getServoController();
        REQUIRE(servo != nullptr);
        REQUIRE(servo->isInitialized() == true);

        // Test steering control
        servo->setAngle(30.0f);
        REQUIRE_THAT(
            system.getVehicle().get_chassis().get_steering_wheel().get_angle(),
            Catch::Matchers::WithinRel(30.0f, 0.001f));
    }

    SECTION("Error Handling Tests")
    {
        // Test invalid config file
        REQUIRE_THROWS_AS(VehicleSystem("invalid_config.json"),
                          std::runtime_error);

        // Test invalid hardware initialization
        auto mock_i2c = std::make_shared<MockI2C>();
        mock_i2c->init("/dev/i2c-invalid");
        REQUIRE_THROWS_AS(VehicleSystem().initHardware(mock_i2c, nullptr),
                          std::runtime_error);
    }
}
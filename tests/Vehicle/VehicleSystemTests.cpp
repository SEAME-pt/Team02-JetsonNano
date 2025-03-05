#include <catch2/catch_test_macros.hpp>
#include "VehicleSystem.hpp"
#include <filesystem>

class MockI2C : public I2C
{
  public:
    bool init(const std::string&) override { return true; }
    bool write(uint8_t, uint8_t) override { return true; }
};

class MockCAN : public CAN
{
  public:
    bool init(const std::string&) override { return true; }
    bool writeMessage(uint32_t, uint8_t*, size_t) override { return true; }
};

TEST_CASE("VehicleSystem Tests", "[vehicle_system]")
{
    SECTION("Default Configuration")
    {
        auto mock_i2c = std::make_shared<MockI2C>();
        auto mock_can = std::make_shared<MockCAN>();

        VehicleSystem system;

        REQUIRE(system.getVehicle().get_is_moving() == false);
        REQUIRE(system.getSession() != nullptr);
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
}
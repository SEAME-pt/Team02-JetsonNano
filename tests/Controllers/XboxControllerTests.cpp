#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "XboxController.hpp"

extern void custom_xbox_setMockData(const std::vector<uint8_t>& data);
extern std::vector<uint8_t> custom_xbox_getWrittenData();

TEST_CASE("XboxController Constructor", "[XboxController]") {
    REQUIRE_NOTHROW(XboxController());
}

TEST_CASE("XboxController Constructor with Config File", "[XboxController]") {
    REQUIRE_NOTHROW(XboxController("configFilePath"));
}

TEST_CASE("XboxController Destructor", "[XboxController]") {
    XboxController* controller = new XboxController();
    REQUIRE_NOTHROW(delete controller);
}

TEST_CASE("XboxController readEvent", "[XboxController]") {
    XboxController controller;
    int result = controller.readEvent();
    REQUIRE(result == -1); // Assuming no event is read
}

TEST_CASE("XboxController getButtonCount", "[XboxController]") {
    XboxController controller;
    int buttonCount = controller.getButtonCount();
    REQUIRE(buttonCount >= 0);
}

TEST_CASE("XboxController getAxisCount", "[XboxController]") {
    XboxController controller;
    int axisCount = controller.getAxisCount();
    REQUIRE(axisCount >= 0);
}

TEST_CASE("XboxController getAxisState", "[XboxController]") {
    XboxController controller;
    int axisState = controller.getAxisState();
    REQUIRE(axisState >= 0);
}

TEST_CASE("XboxController run", "[XboxController]") {
    XboxController controller;
    // This is a more complex test and might require mocking the input events
    // For now, we will just call the method to ensure it runs without crashing
    REQUIRE_NOTHROW(controller.run());
}
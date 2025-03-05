#include <catch2/catch_test_macros.hpp>
#include "ChassisSteeringWheel.hpp"
#include <memory>

class MockObserver : public IVehicleObserver
{
  public:
    float last_angle = 0;
    void onSteeringAngleChanged(float angle) override { last_angle = angle; }
};

TEST_CASE("ChassisSteeringWheel Tests", "[steering]")
{
    ChassisSteeringWheel steering;

    SECTION("Angle Tests")
    {
        steering.set_angle(45);
        REQUIRE(steering.get_angle() == 45);
    }

    SECTION("Observer Tests")
    {
        auto observer = std::make_shared<MockObserver>();
        steering.addObserver(observer);

        steering.set_angle(30);
        REQUIRE(observer->last_angle == 30);
    }
}
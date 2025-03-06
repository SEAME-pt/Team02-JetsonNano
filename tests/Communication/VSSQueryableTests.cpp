#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "VSSQueryable.hpp"
#include "Vehicle.hpp"
#include <thread>
#include <chrono>

TEST_CASE("VSS Queryable Integration Tests", "[vss_queryable]")
{
    auto config = zenoh::Config::create_default();

    std::shared_ptr<zenoh::Session> session = std::make_shared<zenoh::Session>(
        zenoh::Session::open(std::move(config)));
    REQUIRE(session != nullptr);

    Vehicle vehicle;
    VSSQueryable queryable(vehicle, session);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    SECTION("Vehicle Speed Querying")
    {
        int32_t testSpeed = 75;
        vehicle.get_mutable_powertrain().get_mutable_electric_motor().set_speed(
            testSpeed);

        auto on_reply = [testSpeed](const zenoh::Reply& reply)
        {
            auto&& sample = reply.get_ok();
            REQUIRE(std::stoi(sample.get_payload().as_string()) == testSpeed);
        };

        auto on_done = []() { std::cout << "No more replies" << std::endl; };

        session->get("Vehicle/1/Powertrain/ElectricMotor/Speed", "", on_reply,
                     on_done);
    }

    SECTION("Steering Angle Querying")
    {
        float testAngle = 30.0f;
        vehicle.get_mutable_chassis().get_mutable_steering_wheel().set_angle(
            testAngle);

        auto on_reply = [testAngle](const zenoh::Reply& reply)
        {
            auto&& sample = reply.get_ok();
            REQUIRE_THAT(std::stof(sample.get_payload().as_string()),
                         Catch::Matchers::WithinRel(testAngle, 0.001f));
        };

        auto on_done = []() { std::cout << "No more replies" << std::endl; };

        session->get("Vehicle/1/Chassis/SteeringWheel/Angle", "", on_reply,
                     on_done);
    }

    SECTION("Lights Status Querying")
    {
        SECTION("Beam Lights")
        {
            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_beam_low()
                .set_is_on(true);

            auto on_reply1 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done1 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/Beam/Low", "", on_reply1,
                         on_done1);

            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_beam_high()
                .set_is_on(true);

            auto on_reply2 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done2 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/Beam/High", "", on_reply2,
                         on_done2);
        }

        SECTION("Fog Lights")
        {
            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_fog_front()
                .set_is_on(true);

            auto on_reply1 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done1 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/Fog/Front", "", on_reply1,
                         on_done1);

            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_fog_rear()
                .set_is_on(true);

            auto on_reply2 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done2 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/Fog/Rear", "", on_reply2,
                         on_done2);
        }

        SECTION("Brake Lights")
        {
            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_brake()
                .set_is_active(true);

            auto on_reply1 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done1 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/Brake", "", on_reply1,
                         on_done1);
        }

        SECTION("Direction Indicators")
        {
            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_direction_indicator_left()
                .set_is_signaling(true);

            auto on_reply1 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done1 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/DirectionIndicator/Left", "",
                         on_reply1, on_done1);

            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_direction_indicator_right()
                .set_is_signaling(true);

            auto on_reply2 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done2 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/DirectionIndicator/Right", "",
                         on_reply2, on_done2);
        }

        SECTION("Other Lights")
        {
            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_parking()
                .set_is_on(true);

            auto on_reply1 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done1 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/Parking", "", on_reply1,
                         on_done1);

            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_running()
                .set_is_on(true);

            auto on_reply2 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done2 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/Running", "", on_reply2,
                         on_done2);

            vehicle.get_mutable_body()
                .get_mutable_lights()
                .get_mutable_hazard()
                .set_is_signaling(true);

            auto on_reply3 = [](const zenoh::Reply& reply)
            {
                auto&& sample = reply.get_ok();
                REQUIRE(std::stoi(sample.get_payload().as_string()) == 1);
            };

            auto on_done3 = []()
            { std::cout << "No more replies" << std::endl; };

            session->get("Vehicle/1/Body/Lights/Hazard", "", on_reply3,
                         on_done3);
        }
    }
}
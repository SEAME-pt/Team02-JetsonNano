#include <catch2/catch_test_macros.hpp>
#include "Body.hpp"

TEST_CASE("Body Tests", "[body]")
{
    Body body;

    SECTION("Lights Integration Tests")
    {
        Lights lights;

        // Setup lights
        auto& beam_low = lights.get_mutable_beam_low();
        beam_low.set_is_on(true);
        beam_low.set_is_defect(false);

        auto& hazard = lights.get_mutable_hazard();
        hazard.set_is_signaling(true);
        hazard.set_is_defect(false);

        // Test setter
        body.set_lights(lights);

        // Test const getter
        const auto& result = body.get_lights();
        REQUIRE(result.get_beam_low().get_is_on() == true);
        REQUIRE(result.get_beam_low().get_is_defect() == false);
        REQUIRE(result.get_hazard().get_is_signaling() == true);
        REQUIRE(result.get_hazard().get_is_defect() == false);

        // Test mutable getter
        auto& mutable_lights = body.get_mutable_lights();
        mutable_lights.get_mutable_beam_low().set_is_on(false);
        REQUIRE(body.get_lights().get_beam_low().get_is_on() == false);
    }
}
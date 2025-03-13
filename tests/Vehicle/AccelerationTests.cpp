#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "Acceleration.hpp"

TEST_CASE("Acceleration Tests", "[acceleration]")
{
    Acceleration acceleration;

    SECTION("Lateral Acceleration Tests")
    {
        acceleration.set_lateral(2.5f);
        REQUIRE_THAT(acceleration.get_lateral(),
                     Catch::Matchers::WithinRel(2.5f, 0.001f));

        acceleration.set_lateral(-1.8f);
        REQUIRE_THAT(acceleration.get_lateral(),
                     Catch::Matchers::WithinRel(-1.8f, 0.001f));
    }

    SECTION("Longitudinal Acceleration Tests")
    {
        acceleration.set_longitudinal(3.2f);
        REQUIRE_THAT(acceleration.get_longitudinal(),
                     Catch::Matchers::WithinRel(3.2f, 0.001f));

        acceleration.set_longitudinal(-2.1f);
        REQUIRE_THAT(acceleration.get_longitudinal(),
                     Catch::Matchers::WithinRel(-2.1f, 0.001f));
    }

    SECTION("Vertical Acceleration Tests")
    {
        acceleration.set_vertical(1.5f);
        REQUIRE_THAT(acceleration.get_vertical(),
                     Catch::Matchers::WithinRel(1.5f, 0.001f));

        acceleration.set_vertical(-0.9f);
        REQUIRE_THAT(acceleration.get_vertical(),
                     Catch::Matchers::WithinRel(-0.9f, 0.001f));
    }
}
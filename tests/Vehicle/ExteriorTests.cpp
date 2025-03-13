#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "Exterior.hpp"

TEST_CASE("Exterior Tests", "[exterior]")
{
    Exterior exterior;

    SECTION("Air Temperature Tests")
    {
        exterior.set_air_temperature(25.5f);
        REQUIRE_THAT(exterior.get_air_temperature(),
                     Catch::Matchers::WithinRel(25.5f, 0.001f));
    }

    SECTION("Humidity Tests")
    {
        exterior.set_humidity(65.0f);
        REQUIRE_THAT(exterior.get_humidity(),
                Catch::Matchers::WithinRel(65.0f, 0.001f));
    }

    SECTION("Light Intensity Tests")
    {
        exterior.set_light_intensity(1000.0f);
        REQUIRE_THAT(exterior.get_light_intensity(),
                Catch::Matchers::WithinRel(1000.0f, 0.001f));
    }
}
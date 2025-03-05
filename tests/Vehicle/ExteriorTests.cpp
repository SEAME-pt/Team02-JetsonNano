#include <catch2/catch_test_macros.hpp>
#include "Exterior.hpp"

TEST_CASE("Exterior Tests", "[exterior]")
{
    Exterior exterior;

    SECTION("Air Temperature Tests")
    {
        exterior.set_air_temperature(25.5f);
        REQUIRE(exterior.get_air_temperature() == Approx(25.5f));
    }

    SECTION("Humidity Tests")
    {
        exterior.set_humidity(65.0f);
        REQUIRE(exterior.get_humidity() == Approx(65.0f));
    }

    SECTION("Light Intensity Tests")
    {
        exterior.set_light_intensity(1000.0f);
        REQUIRE(exterior.get_light_intensity() == Approx(1000.0f));
    }
}
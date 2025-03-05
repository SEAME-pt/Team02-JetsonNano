#include <catch2/catch_test_macros.hpp>
#include "Connectivity.hpp"

TEST_CASE("Connectivity Tests", "[connectivity]")
{
    Connectivity connectivity;

    SECTION("Connectivity Availability Tests")
    {
        connectivity.set_is_connectivity_available(true);
        REQUIRE(connectivity.get_is_connectivity_available() == true);

        connectivity.set_is_connectivity_available(false);
        REQUIRE(connectivity.get_is_connectivity_available() == false);
    }
}
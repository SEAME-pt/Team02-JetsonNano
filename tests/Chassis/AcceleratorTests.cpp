#include <catch2/catch_test_macros.hpp>
#include "Accelerator.hpp"

TEST_CASE("Accelerator Tests", "[accelerator]")
{
    Accelerator accelerator;

    SECTION("Pedal Position Tests")
    {
        accelerator.set_pedal_position(0);
        REQUIRE(accelerator.get_pedal_position() == 0);

        accelerator.set_pedal_position(100);
        REQUIRE(accelerator.get_pedal_position() == 100);
    }
}
#include "pidCalibrator.hpp"

#include <iostream>

int main(int argc, char* argv[])
{
    // Initialize Zenoh session
    auto config  = zenoh::Config::create_default();
    auto session = std::make_shared<zenoh::Session>(
        zenoh::Session::open(std::move(config)));
    if (!session)
    {
        std::cerr << "Failed to open Zenoh session." << std::endl;
        return 1;
    }

    // Create PID calibrator instance
    PidCalibrator pidCalibrator(session);

    // Run the PID calibrator
    pidCalibrator.run();

    // Close Zenoh session
    session->close();

    return 0;
}
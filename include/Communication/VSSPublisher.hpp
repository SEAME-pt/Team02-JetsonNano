#pragma once

#include "Vehicle.hpp"
#include <zenoh.hxx>
#include <optional>

class VSSPublisher
{
  public:
    VSSPublisher(Vehicle& vehicle);

  private:
    std::unique_ptr<zenoh::Session> session;
    std::optional<zenoh::Publisher> throttle_pub;
    std::optional<zenoh::Publisher> steering_pub;
    std::optional<zenoh::Publisher> beamLow_pub;
    std::optional<zenoh::Publisher> beamHigh_pub;
    std::optional<zenoh::Publisher> running_pub;
    std::optional<zenoh::Publisher> parking_pub;
    std::optional<zenoh::Publisher> fogRear_pub;
    std::optional<zenoh::Publisher> fogFront_pub;
    std::optional<zenoh::Publisher> brake_pub;
    std::optional<zenoh::Publisher> hazard_pub;
    std::optional<zenoh::Publisher> directionIndicatorLeft_pub;
    std::optional<zenoh::Publisher> directionIndicatorRight_pub;
};

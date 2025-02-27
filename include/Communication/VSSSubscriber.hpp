#pragma once

#include "Vehicle.hpp"
#include <zenoh.hxx>
#include <optional>

class VSSSubscriber
{
  public:
    VSSSubscriber(Vehicle& vehicle);

  private:
    Vehicle& vehicle_;
    std::unique_ptr<zenoh::Session> session;
    std::optional<zenoh::Subscriber<void>> throttle_subscriber;
    std::optional<zenoh::Subscriber<void>> steering_subscriber;
    std::optional<zenoh::Subscriber<void>> beamLow_subscriber;
    std::optional<zenoh::Subscriber<void>> beamHigh_subscriber;
    std::optional<zenoh::Subscriber<void>> running_subscriber;
    std::optional<zenoh::Subscriber<void>> parking_subscriber;
    std::optional<zenoh::Subscriber<void>> fogRear_subscriber;
    std::optional<zenoh::Subscriber<void>> fogFront_subscriber;
    std::optional<zenoh::Subscriber<void>> brake_subscriber;
    std::optional<zenoh::Subscriber<void>> hazard_subscriber;
    std::optional<zenoh::Subscriber<void>> directionIndicatorLeft_subscriber;
    std::optional<zenoh::Subscriber<void>> directionIndicatorRight_subscriber;

    void setupSubscriptions();
};
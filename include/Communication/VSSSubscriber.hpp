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

    void setupSubscriptions();
};
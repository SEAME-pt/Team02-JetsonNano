#pragma once

#include "Vehicle.hpp"
#include <zenoh.hxx>
#include <optional>

class VSSSubscriber
{
  public:
    VSSSubscriber(Vehicle& vehicle, std::shared_ptr<zenoh::Session> session);

    // Constructor with callback to send messages to CAN.
    VSSSubscriber(Vehicle& vehicle,
                  std::function<void(uint32_t, uint8_t*, size_t)> sendToCAN,
                  std::shared_ptr<zenoh::Session> session);

  private:
    Vehicle& vehicle_;
    std::shared_ptr<zenoh::Session> session_;

    std::function<void(uint32_t, uint8_t*, size_t)> sendToCAN_;
    uint8_t lights_[1] = {0};

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
    std::optional<zenoh::Subscriber<void>> stateOfCharge_subscriber;
    std::optional<zenoh::Subscriber<void>> maxVoltage_subscriber;
    std::optional<zenoh::Subscriber<void>> currentVoltage_subscriber;
    std::optional<zenoh::Subscriber<void>> currentCurrent_subscriber;
    std::optional<zenoh::Subscriber<void>> currentPower_subscriber;
    std::optional<zenoh::Subscriber<void>> currentGear_subscriber;

    void setupSubscriptions();
};
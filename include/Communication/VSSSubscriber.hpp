#pragma once

#include "Vehicle.hpp"
#include <zenoh.hxx>
#include <optional>

/**
 * @brief Vehicle Signal Specification (VSS) subscriber
 *
 * @details Subscribes to VSS-compliant vehicle control signals including:
 *          - Motion control (throttle, steering)
 *          - Light control (all exterior lights)
 *          - Battery monitoring
 *          - Gear selection
 *
 * Features:
 * - Automatic signal routing to vehicle systems
 * - CAN bus integration support
 * - Real-time state updates
 *
 * Signal categories:
 * - Vehicle control (speed, steering)
 * - Lighting system control
 * - Powertrain monitoring
 * - Battery management
 *
 * @note Implements VSS data model for vehicle signals
 * @see Vehicle
 * @see zenoh::Session
 * @see zenoh::Subscriber
 */
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
    std::optional<zenoh::Subscriber<void>> speed_subscriber;
    std::optional<zenoh::Subscriber<void>> currentVoltage_subscriber;
    std::optional<zenoh::Subscriber<void>> currentCurrent_subscriber;
    std::optional<zenoh::Subscriber<void>> currentPower_subscriber;
    std::optional<zenoh::Subscriber<void>> currentGear_subscriber;

    void setupSubscriptions();
};
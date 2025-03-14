#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include "CAN.hpp"
#include <arpa/inet.h>
#include "VehicleFactory.hpp"
#include "VSSSubscriber.hpp"
#include "VSSQueryable.hpp"
#include "SensoringPublisher.hpp"

using namespace zenoh;

/**
 * @brief Vehicle signals management and speed monitoring system
 *
 * @details Manages CAN bus signals and processes vehicle data:
 *          - Speed calculation from wheel rotation
 *          - CAN message processing (ID 0x01)
 *          - Real-time speed publishing
 *          - Signal validation and filtering
 *
 * Calculations:
 * - Speed = wheelDiameter * π * rotationSpeed * 10 / 60
 * - Wheel diameter: 67mm
 * - Valid speed range: 0-100
 *
 * Message format:
 * - CAN ID: 0x01
 * - Data[1-4]: Speed value (network byte order)
 *
 * @note Requires CAN bus interface
 * @note Publishes speed updates every 300ms
 * @see CAN
 * @see SensoringPublisher
 */
class Signals
{
  private:
    CAN* canBus;
    std::shared_ptr<SensoringPublisher> publisher_;

  public:
    Signals(std::shared_ptr<SensoringPublisher> publisher);
    ~Signals();

    void init(const std::string& canDevice);
    void run();
};

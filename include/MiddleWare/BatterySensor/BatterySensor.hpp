
#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include "I2C.hpp"
#include "INA219.hpp"
#include "CAN.hpp"
#include "SensoringPublisher.hpp"

using namespace zenoh;

extern int signalTo;

/**
 * @brief Battery monitoring system using INA219
 *
 * @details Monitors and processes battery parameters:
 *          - Voltage measurement via INA219
 *          - Voltage smoothing with EMA filter
 *          - State of charge calculation
 *          - CAN message transmission
 *          - Data publishing via Zenoh
 *
 * Specifications:
 * - Voltage range: 9.5V - 12.6V
 * - Sample rate: 100ms
 * - EMA alpha: 0.01
 * - CAN message ID: 0x02
 *
 * Calculations:
 * - State of charge = ((voltage - 9.5) / (12.6 - 9.5)) * 100%
 * - Voltage change threshold: 0.04V
 *
 * Hardware interfaces:
 * - I2C for INA219 sensor
 * - CAN for data transmission (8 bytes)
 * - Zenoh for state publishing
 *
 * @note Implements voltage spike filtering
 * @note Clamps SoC to 0-100%
 * @see I2C
 * @see INA219
 * @see CAN
 * @see SensoringPublisher
 */
class BatterySensor
{
  private:
    I2C* m_I2c;
    INA219* batteryINA;
    CAN* canBus;
    std::shared_ptr<SensoringPublisher> publisher_;

  public:
    BatterySensor(std::shared_ptr<SensoringPublisher> publisher);
    ~BatterySensor();

    void init(const std::string& i2cDevice, uint8_t sensorAddress,
              const std::string& canDevice);
    void run(void);
};

#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>

/**
 * @brief Publisher for vehicle sensor data
 *
 * @details Publishes vehicle sensor readings via Zenoh including:
 *          - Vehicle speed
 *          - Battery voltage
 *          - Battery current
 *          - Battery power
 *          - Battery state of charge
 *
 * Each measurement is published on a dedicated Zenoh topic for:
 * - Real-time monitoring
 * - Data logging
 * - External system integration
 *
 * @note All electrical measurements use SI units (V, A, W)
 * @see zenoh::Session
 * @see zenoh::Publisher
 */
class SensoringPublisher
{
  public:
    SensoringPublisher(std::shared_ptr<zenoh::Session> session);

    void publishSpeed(float speed);
    void publishCurrentVoltage(float voltage);
    void publishCurrentCurrent(float current);
    void publishCurrentPower(float power);
    void publishStateOfCharge(float state_of_charge);

  private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::Publisher> speed_pub;
    std::optional<zenoh::Publisher> current_voltage_pub;
    std::optional<zenoh::Publisher> current_current_pub;
    std::optional<zenoh::Publisher> current_power_pub;
    std::optional<zenoh::Publisher> state_of_charge_pub;
};

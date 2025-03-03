#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>

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

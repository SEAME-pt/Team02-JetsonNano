#pragma once

#include "Vehicle.hpp"
#include "zenoh.hxx"

class VSSSubscriber
{
  public:
    VSSSubscriber(Vehicle& vehicle) : vehicle_(vehicle)
    {
        // Setup Zenoh subscriber
        setupZenohSubscription();
    }

  private:
    Vehicle& vehicle_;
    zenoh::Session session_;

    void setupZenohSubscription()
    {
        session_.declare_subscriber("vehicle/control/throttle")
            .callback(
                [this](const zenoh::Sample& sample)
                {
                    float throttle = std::stof(sample.get_value().as_string());
                    this->vehicle_.get_mutable_powertrain()
                        .get_mutable_electric_motor()
                        .set_speed(convertThrottleToSpeed(throttle));
                });
    }
};
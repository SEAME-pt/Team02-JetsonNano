#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>

class XboxControllerPublisher
{
  public:
    XboxControllerPublisher();

    void publishSpeed(float speed);
    void publishSteering(float steering);

  private:
    std::unique_ptr<zenoh::Session> session;
    std::optional<zenoh::Publisher> throttle_pub;
    std::optional<zenoh::Publisher> steering_pub;
};
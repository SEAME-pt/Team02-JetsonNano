#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>

class ControllerPublisher
{
  public:
    ControllerPublisher(std::shared_ptr<zenoh::Session> session);

    void publishSpeed(float speed);
    void publishSteering(float steering);
    void publishBeamLow(bool isOn);
    void publishBeamHigh(bool isOn);
    void publishRunning(bool isOn);
    void publishParking(bool isOn);
    void publishFogRear(bool isOn);
    void publishFogFront(bool isOn);
    void publishBrake(bool isActive);
    void publishHazard(bool isSignaling);
    void publishDirectionIndicatorLeft(bool isSignaling);
    void publishDirectionIndicatorRight(bool isSignaling);
    void publishCurrentGear(int gear);

  private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::Publisher> throttle_pub;
    std::optional<zenoh::Publisher> steering_pub;
    std::optional<zenoh::Publisher> beamLow_pub;
    std::optional<zenoh::Publisher> beamHigh_pub;
    std::optional<zenoh::Publisher> running_pub;
    std::optional<zenoh::Publisher> parking_pub;
    std::optional<zenoh::Publisher> fogRear_pub;
    std::optional<zenoh::Publisher> fogFront_pub;
    std::optional<zenoh::Publisher> brake_pub;
    std::optional<zenoh::Publisher> hazard_pub;
    std::optional<zenoh::Publisher> directionIndicatorLeft_pub;
    std::optional<zenoh::Publisher> directionIndicatorRight_pub;
    std::optional<zenoh::Publisher> currentGear_pub;
};
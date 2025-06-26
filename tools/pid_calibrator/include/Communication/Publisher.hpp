#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>

/**
 * @brief Controller state publisher
 *
 * @details Publishes controller va:
 *          - Motion control (speed, steering)
 *          - Light system controls
 *          - Gear selection
 *          - Warning signals
 *
 * Published states:
 * - Speed control (-100 to +100)
 * - Steering angle (0° to 180°)
 * - Light states (on/off)
 * - Gear selection (P,R,N,D)
 * - Warning indicators (hazard, direction)
 *
 * @note All states published as VSS-compliant signals
 * @see zenoh::Session
 * @see zenoh::Publisher
 * @see XboxController
 */
class Publisher
{
  public:
    Publisher(std::shared_ptr<zenoh::Session> session);

    void publishSpeedKp(float kp);
    void publishSpeedKi(float ki);
    void publishSpeedKd(float kd);

  private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;
    std::optional<zenoh::Publisher> SpeedKp_pub;
    std::optional<zenoh::Publisher> SpeedKi_pub;
    std::optional<zenoh::Publisher> SpeedKd_pub;
};
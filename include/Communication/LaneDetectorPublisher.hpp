#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>

/**
 * @brief Lane detector publisher
 *
 * @details Publishes lane detection data and status information:
 *          - Camera error information
 *
 * Published data:
 * - Camera error values
 *
 * @note Uses Zenoh for publishing lane detection related information
 * @see zenoh::Session
 * @see zenoh::Publisher
 */

class LaneDetectorPublisher
{
  public:
    LaneDetectorPublisher(std::shared_ptr<zenoh::Session> session);

    void publishCameraError(float speed);

  private:
    zenoh::PosixShmProvider provider_;
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::Publisher> cameraError_pub;
};
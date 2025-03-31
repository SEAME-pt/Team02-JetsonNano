#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <sys/time.h>
#include <iomanip>

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
    void publishLanes(const std::vector<cv::Point>& leftLane, const std::vector<cv::Point>& rightLane);
    void publishCameraFrame(cv::Mat frame);
    double getCurrentTime();

  private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;
    std::optional<zenoh::Publisher> cameraError_pub;
    std::optional<zenoh::Publisher> cameraLanes_pub;
    std::optional<zenoh::Publisher> cameraFrame_pub;

    int frame_count_ = 0;
    double last_frame_time_ = 0.0;
    const double target_fps_ = 15.0;
};
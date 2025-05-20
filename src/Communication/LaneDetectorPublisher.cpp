#include <LaneDetectorPublisher.hpp>

// In LaneDetectorPublisher.cpp
double LaneDetectorPublisher::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

LaneDetectorPublisher::LaneDetectorPublisher(
    std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));

    cameraError_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/LaneDetection/CameraError")));
    cameraFrame_pub.emplace(
        session_->declare_publisher(zenoh::KeyExpr("video/stream")));
}

void LaneDetectorPublisher::publishCameraError(float error)
{
    std::cout << "Error send: " << error << std::endl;
    std::string value_str = std::to_string(error);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    cameraError_pub->put(std::move(buf));
}

void LaneDetectorPublisher::publishCameraFrame(cv::Mat frame)
{
    // Throttle FPS to avoid network congestion
    double current_time = getCurrentTime(); // Use your existing time function
    if (current_time - last_frame_time_ < 1.0 / target_fps_)
    {
        return;
    }
    last_frame_time_ = current_time;

    // Resize and compress frame for network transmission
    cv::Mat resized_frame;
    cv::resize(frame, resized_frame, cv::Size(640, 360)); // Smaller resolution

    // JPEG compression (adjust quality as needed)
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80}; // 80% quality
    cv::imencode(".jpg", resized_frame, buffer, params);

    std::cout << "send frame" << std::endl;
    cameraFrame_pub->put(buffer);
}

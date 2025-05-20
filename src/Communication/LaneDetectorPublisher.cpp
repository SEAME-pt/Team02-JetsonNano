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
    cameraLanes_pub.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/LaneData")));
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

    std::cout << "frame ready" << std::endl;
    cameraFrame_pub->put(buffer);
}

void LaneDetectorPublisher::publishLanes(const cv::Mat& leftLane,
                                         const cv::Mat& rightLane)
{
    // Extract coefficients
    float leftA =
        static_cast<float>(leftCoeffs.at<double>(0)); // quadratic coefficient
    float leftB =
        static_cast<float>(leftCoeffs.at<double>(1)); // linear coefficient
    float leftC = static_cast<float>(leftCoeffs.at<double>(2)); // constant term
    float rightA =
        static_cast<float>(rightCoeffs.at<double>(0)); // quadratic coefficient
    float rightB =
        static_cast<float>(rightCoeffs.at<double>(1)); // linear coefficient
    float rightC =
        static_cast<float>(rightCoeffs.at<double>(2)); // constant term

    // Compute midline coefficients (simple average of left and right)
    float midA = (leftA + rightA) / 2.0f;
    float midB = (leftB + rightB) / 2.0f;
    float midC = (leftC + rightC) / 2.0f;
    std::cout << "Midline polynomial: " << midA << "y² + " << midB << "y + "
              << midC << std::endl;

    ss << midA << " " << midB << " " << midC;
    std::string laneData = ss.str();

    // Publish lane data
    const auto len = laneData.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), laneData.c_str(), len);
    cameraLanes_pub->put(std::move(buf));
}
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
    if (current_time - last_frame_time_ < 1.0/target_fps_) {
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
    
    // Create timestamp and metadata
    frame_count_++;
    std::string metadata = "frame_" + std::to_string(frame_count_) + 
                           "_time_" + std::to_string(current_time);
    
    // Create Zenoh payload with metadata prefix
    std::vector<uint8_t> payload(metadata.begin(), metadata.end());
    payload.push_back(':'); // Separator
    payload.insert(payload.end(), buffer.begin(), buffer.end());
    cameraError_pub->put(payload);
}

void LaneDetectorPublisher::publishLanes(
    const std::vector<cv::Point>& leftLane,
    const std::vector<cv::Point>& rightLane)
{
    std::stringstream ss;

    // Select three key points from each lane (bottom, middle, top)
    std::vector<cv::Point> leftKeyPoints, rightKeyPoints;

    if (!leftLane.empty())
    {
        int bottom_idx = 0; // Bottom point (closest to vehicle)
        int mid_idx    = leftLane.size() / 2; // Middle point
        int top_idx = leftLane.size() - 1; // Top point (furthest from vehicle)

        leftKeyPoints = {leftLane[bottom_idx], leftLane[mid_idx],
                         leftLane[top_idx]};
    }

    if (!rightLane.empty())
    {
        int bottom_idx = 0;
        int mid_idx    = rightLane.size() / 2;
        int top_idx    = rightLane.size() - 1;

        rightKeyPoints = {rightLane[bottom_idx], rightLane[mid_idx],
                          rightLane[top_idx]};
    }

    // Format left lane points
    ss << "leftLane:";
    for (const auto& point : leftKeyPoints)
    {
        ss << " " << point.x << "," << point.y;
    }

    // Add separator and format right lane points
    ss << "\nrightLane:";
    for (const auto& point : rightKeyPoints)
    {
        ss << " " << point.x << "," << point.y;
    }

    std::string laneData = ss.str();
    std::cout << "Publishing lanes: " << laneData << std::endl;

    // Publish lane data
    const auto len = laneData.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), laneData.c_str(), len);
    cameraLanes_pub->put(std::move(buf));
}
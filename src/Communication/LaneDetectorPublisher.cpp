#include <LaneDetectorPublisher.hpp>

LaneDetectorPublisher::LaneDetectorPublisher(
    std::shared_ptr<zenoh::Session> session)
{
    session_ = session;

    cameraError_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/LaneDetection/CameraError")));
    cameraLanes_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/LaneData")));
}

void LaneDetectorPublisher::publishCameraError(float error)
{
    std::cout << "Error send: " << error << std::endl;
    cameraError_pub->put(std::to_string(error));
}

void LaneDetectorPublisher::publishLanes(const std::vector<cv::Point>& leftLane, const std::vector<cv::Point>& rightLane)
{
    std::stringstream ss;
    
    // Select three key points from each lane (bottom, middle, top)
    std::vector<cv::Point> leftKeyPoints, rightKeyPoints;
    
    if (!leftLane.empty()) {
        int bottom_idx = 0;  // Bottom point (closest to vehicle)
        int mid_idx = leftLane.size() / 2;  // Middle point
        int top_idx = leftLane.size() - 1;  // Top point (furthest from vehicle)
        
        leftKeyPoints = {leftLane[bottom_idx], leftLane[mid_idx], leftLane[top_idx]};
    }
    
    if (!rightLane.empty()) {
        int bottom_idx = 0;
        int mid_idx = rightLane.size() / 2;
        int top_idx = rightLane.size() - 1;
        
        rightKeyPoints = {rightLane[bottom_idx], rightLane[mid_idx], rightLane[top_idx]};
    }
    
    // Format left lane points
    ss << "leftLane:";
    for (const auto& point : leftKeyPoints) {
        ss << " " << point.x << "," << point.y;
    }
    
    // Add separator and format right lane points
    ss << "\nrightLane:";
    for (const auto& point : rightKeyPoints) {
        ss << " " << point.x << "," << point.y;
    }
    
    std::string laneData = ss.str();
    std::cout << "Publishing lanes: " << laneData << std::endl;
    
    // Publish lane data
    cameraLanes_pub->put(laneData);
}
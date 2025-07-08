#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <sys/time.h>
#include <iostream>

class AdaptiveCruiseControl {
public:
    AdaptiveCruiseControl(int frameWidth, int frameHeight);
    ~AdaptiveCruiseControl();

    float calculateAdaptiveSpeed(const cv::Mat& segmentationMask, 
                                const std::vector<cv::Point>& midCurve);
    
    // Getters for visualization
    int getCurrentObstacleDistance() const { return currentObstacleDistance_; }
    float getObstacleSpeed() const { return obstacleSpeed_; }
    bool isObstacleDetected() const { return obstacleDetected_; }
    cv::Point getObstaclePosition() const { return obstaclePosition_; }

private:
    // Frame dimensions
    int frameWidth_;
    int frameHeight_;
    float nearDistance_;
    float farDistance_;
    float laneWidth_;
    
    // Obstacle tracking
    struct ObstacleInfo {
        int distance;
        cv::Point position;
        double timestamp;
    };
    
    std::deque<ObstacleInfo> obstacleHistory_;
    const int MAX_HISTORY = 5;
    
    // Current state
    int currentObstacleDistance_;
    int previousObstacleDistance_;
    cv::Point obstaclePosition_;
    float obstacleSpeed_;  // pixels per second
    bool obstacleDetected_;
    double lastMeasurementTime_;

    //Calculating object speed
    float currentSpeed_ = 0.0f;
    float desiredSpeed_ = 0.0f;
    
    // Detection parameters
    const int DETECTION_ZONE_WIDTH = 60;  // pixels on each side of trajectory
    const float IGNORE_ZONE_RATIO = 0.9f; // ignore bottom 80% of frame
    
    // Helper functions
    int findObstacleOnTrajectory(const cv::Mat& segmentationMask, 
                                const std::vector<cv::Point>& midCurve);
    void updateObstacleTracking(int obstacleDistance, const cv::Point& obstaclePos);
    float calculateObstacleSpeed();
    bool isRoadPixel(const cv::Vec3b& pixel);
    double getCurrentTime();
};
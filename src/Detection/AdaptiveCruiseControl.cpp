#include "AdaptiveCruiseControl.hpp"
#include <algorithm>
#include <cmath>

AdaptiveCruiseControl::AdaptiveCruiseControl(std::shared_ptr<zenoh::Session> session, int frameWidth, int frameHeight, float nearDistance, float farDistance, float laneWidth)
    : session_(session),
      frameWidth_(frameWidth), frameHeight_(frameHeight),
      nearDistance_(nearDistance), farDistance_(farDistance),
      laneWidth_(laneWidth),
      currentObstacleDistance_(-1), previousObstacleDistance_(-1),
      obstaclePosition_(cv::Point(-1, -1)), obstacleSpeed_(0.0f),
      obstacleDetected_(false), lastMeasurementTime_(0.0)
{
    // Initialize Zenoh subscribers
    currentSpeed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed",
        [this](const zenoh::Sample& sample)
        {
            float speed = std::stof(sample.get_payload().as_string());
            currentSpeed_ = speed / 60.0 * (M_PI * 0.067);
        },
        zenoh::closures::none));
    
    desiredSpeed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/speedPid/DesiredSpeed",
        [this](const zenoh::Sample& sample)
        {
            float speed = std::stof(sample.get_payload().as_string());
            desiredSpeed_ = speed;
        },
        zenoh::closures::none));
}

AdaptiveCruiseControl::~AdaptiveCruiseControl()
{}

float AdaptiveCruiseControl::calculateAdaptiveSpeed(const cv::Mat& segmentationMask, 
                                                   const std::vector<cv::Point>& midCurve)
{
    if (midCurve.empty() || segmentationMask.empty()) {
        return 1.0f; // Full speed if no data
    }
    
    // Find obstacle distance along trajectory
    int obstacleDistance = findObstacleOnTrajectory(segmentationMask, midCurve);
    std::cout << "OBSTACLE DISTANCE : " << obstacleDistance << std::endl;
    
    // Update tracking history
    cv::Point obstaclePos = (obstacleDistance > 0) ? 
        cv::Point(frameWidth_/2, frameHeight_ - obstacleDistance) : cv::Point(-1, -1);
    updateObstacleTracking(obstacleDistance, obstaclePos);
    
    // Calculate obstacle speed from history
    obstacleSpeed_ = calculateObstacleSpeed();
    
    if (!obstacleDetected_) {
        return desiredSpeed_; // Full speed - no obstacle
    }
    
    float obstacleDistanceMeters = static_cast<float>(currentObstacleDistance_) * 
                                  (farDistance_ - nearDistance_) / frameHeight_;
    
    // Define safety distances in meters
    const float CRITICAL_DISTANCE_M = 0.3f;
    const float SAFE_DISTANCE_M = 0.6f;
    const float COMFORT_DISTANCE_M = 0.9f;

    std::cout << "Current Speed: " << currentSpeed_ << " Obstacle Speed: " << obstacleSpeed_ << std::endl;
    
    // Calculate absolute speed of obstacle (in m/s)
    float obstacleAbsoluteSpeed = currentSpeed_ + obstacleSpeed_; // obstacleSpeed_ is relative
    
    // Make sure obstacle speed is not negative (stationary at minimum)
    obstacleAbsoluteSpeed = std::max(0.0f, obstacleAbsoluteSpeed);
    
    if (obstacleDistanceMeters < CRITICAL_DISTANCE_M) {
        // Emergency stop
        std::cout << "EMERGENCY: Obstacle at " << obstacleDistanceMeters << "m - STOPPING" << std::endl;
        return 0.0f;
    }
    else if (obstacleDistanceMeters < SAFE_DISTANCE_M) {
        // Close following: match obstacle speed but reduce slightly for safety
        float targetSpeed = obstacleAbsoluteSpeed * 0.8f; // 80% of obstacle speed
        std::cout << "CLOSE FOLLOW: Obstacle at " << obstacleDistanceMeters << "m, "
                  << "Obstacle speed: " << obstacleAbsoluteSpeed << "m/s, "
                  << "Target: " << targetSpeed << "m/s" << std::endl;
        return targetSpeed;
    }
    else if (obstacleDistanceMeters < COMFORT_DISTANCE_M) {
        // Normal following: match obstacle speed exactly
        std::cout << "FOLLOWING: Obstacle at " << obstacleDistanceMeters << "m, "
                  << "Matching speed: " << obstacleAbsoluteSpeed << "m/s" << std::endl;
        return obstacleAbsoluteSpeed;
    }
    else {
        // Far enough: gradually transition to desired cruise speed
        float blendFactor = (obstacleDistanceMeters - COMFORT_DISTANCE_M) / 
                           (COMFORT_DISTANCE_M * 0.5f); // Blend over 6m distance
        blendFactor = std::clamp(blendFactor, 0.0f, 1.0f);
        
        float targetSpeed = obstacleAbsoluteSpeed * (1.0f - blendFactor) + 
                           desiredSpeed_ * blendFactor;
        
        std::cout << "TRANSITION: Distance " << obstacleDistanceMeters << "m, "
                  << "Blending to cruise speed: " << targetSpeed << "m/s" << std::endl;
        return targetSpeed;
    }
}

int AdaptiveCruiseControl::findObstacleOnTrajectory(const cv::Mat& segmentationMask, 
                                                   const std::vector<cv::Point>& midCurve)
{
    if (segmentationMask.channels() != 3) {
        std::cout << "Invalid segmentation mask format" << std::endl;
        return -1;
    }
    
    // Start from the farthest point and work towards the car
    for (int i = 0; i < static_cast<int>(midCurve.size()); i++) {
        cv::Point trajPoint = midCurve[i];
        
        // Skip points too close to bottom (ignore zone)
        if (trajPoint.y > frameHeight_ * IGNORE_ZONE_RATIO) continue;
        
        // Check trajectory point and surrounding area
        int leftX = std::max(0, trajPoint.x - DETECTION_ZONE_WIDTH);
        int rightX = std::min(frameWidth_ - 1, trajPoint.x + DETECTION_ZONE_WIDTH);
        
        int nonRoadPixels = 0;
        int totalPixels = 0;
        
        // Check horizontal line across detection zone
        for (int x = leftX; x <= rightX; x++) {
            if (trajPoint.y >= 0 && trajPoint.y < frameHeight_ && 
                x >= 0 && x < frameWidth_) {
                
                cv::Vec3b pixel = segmentationMask.at<cv::Vec3b>(trajPoint.y, x);
                totalPixels++;
                
                if (!isRoadPixel(pixel)) {
                    nonRoadPixels++;
                }
            }
        }
        
        // If more than 30% of the detection zone is non-road, consider it an obstacle
        if (totalPixels > 0 && (static_cast<float>(nonRoadPixels) / totalPixels) > 0.1f) {
            return frameHeight_ - trajPoint.y;
        }
    }
    
    return -1; // No obstacle found
}

void AdaptiveCruiseControl::updateObstacleTracking(int obstacleDistance, const cv::Point& obstaclePos)
{
    double currentTime = getCurrentTime();
    float obstacleDistanceMeters = static_cast<float>(obstacleDistance) * (farDistance_ - nearDistance_) / frameHeight_;
    
    
    // Update current state
    previousObstacleDistance_ = currentObstacleDistance_;
    currentObstacleDistance_ = obstacleDistance;
    obstaclePosition_ = obstaclePos;
    obstacleDetected_ = (obstacleDistance > 0);
    
    // Add to history if obstacle detected
    if (obstacleDetected_) {
        ObstacleInfo info;
        info.distance = obstacleDistanceMeters;
        info.position = obstaclePos;
        info.timestamp = currentTime;
        
        obstacleHistory_.push_back(info);
        
        // Keep only recent history
        while (obstacleHistory_.size() > static_cast<size_t>(MAX_HISTORY)) {
            obstacleHistory_.pop_front();
        }
        
        // Remove old entries (older than 2 seconds)
        while (!obstacleHistory_.empty() && 
               (currentTime - obstacleHistory_.front().timestamp) > 2.0) {
            obstacleHistory_.pop_front();
        }
    } else {
        // Clear history if no obstacle detected for a while
        if (currentTime - lastMeasurementTime_ > 0.5) {
            obstacleHistory_.clear();
        }
    }
    
    lastMeasurementTime_ = currentTime;
}

float AdaptiveCruiseControl::calculateObstacleSpeed()
{
    if (obstacleHistory_.size() < 2) {
        return 0.0f;
    }
    
    // Use linear regression over the recent history
    double sumTime = 0.0, sumDist = 0.0, sumTimeDist = 0.0, sumTimeSquared = 0.0;
    int count = 0;
    
    double baseTime = obstacleHistory_.front().timestamp;
    
    for (const auto& info : obstacleHistory_) {
        double t = info.timestamp - baseTime;
        double d = static_cast<double>(info.distance);
        
        sumTime += t;
        sumDist += d;
        sumTimeDist += t * d;
        sumTimeSquared += t * t;
        count++;
    }
    
    if (count < 2 || sumTimeSquared == 0) return 0.0f;
    
    // Calculate slope (speed)
    double slope = (count * sumTimeDist - sumTime * sumDist) / 
                   (count * sumTimeSquared - sumTime * sumTime);

    float obstacleAbsoluteSpeed = currentSpeed_ - static_cast<float>(slope);
    float relativeSpeed = obstacleAbsoluteSpeed - currentSpeed_;

    std::cout << "Obstacle speed: " << obstacleAbsoluteSpeed << "m/s, "
          << "Relative: " << relativeSpeed << "m/s" << std::endl;
    
    return relativeSpeed; // Positive = moving away, Negative = approaching
}

bool AdaptiveCruiseControl::isRoadPixel(const cv::Vec3b& pixel)
{

    // Check for specific road color in your segmentation
    if (std::abs(pixel[0] - 128) == 0 && 
        std::abs(pixel[1] - 64) == 0 && 
        std::abs(pixel[2] - 128) == 0) {
        return true;
    }

    return false; // Non-road pixel
}

double AdaptiveCruiseControl::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}
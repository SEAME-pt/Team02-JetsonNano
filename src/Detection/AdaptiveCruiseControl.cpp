#include "AdaptiveCruiseControl.hpp"
#include <algorithm>
#include <cmath>

AdaptiveCruiseControl::AdaptiveCruiseControl(
    std::shared_ptr<zenoh::Session> session, int frameWidth, int frameHeight,
    float nearDistance, float farDistance, float laneWidth)
    : session_(session), frameWidth_(frameWidth), frameHeight_(frameHeight),
      nearDistance_(nearDistance), farDistance_(farDistance),
      laneWidth_(laneWidth), currentObstacleDistance_(-1),
      previousObstacleDistance_(-1), obstaclePosition_(cv::Point(-1, -1)),
      obstacleSpeed_(0.0f), obstacleDetected_(false), lastMeasurementTime_(0.0)
{
    // Initialize Zenoh subscribers
    currentSpeed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed",
        [this](const zenoh::Sample& sample)
        {
            float speed   = std::stof(sample.get_payload().as_string());
            currentSpeed_ = speed / 60.0 * (M_PI * 0.067);
        },
        zenoh::closures::none));

    desiredSpeed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/speedPid/DesiredSpeed",
        [this](const zenoh::Sample& sample)
        {
            float speed   = std::stof(sample.get_payload().as_string());
            desiredSpeed_ = speed;
        },
        zenoh::closures::none));
}

AdaptiveCruiseControl::~AdaptiveCruiseControl() {}

float AdaptiveCruiseControl::calculateAdaptiveSpeed(
    const cv::Mat& segmentationMask, const std::vector<cv::Point>& midCurve)
{
    if (midCurve.empty() || segmentationMask.empty())
    {
        return -1; // Full speed if no data
    }

    // Find obstacle distance along trajectory
    int obstacleDistance = findObstacleOnTrajectory(segmentationMask, midCurve);

    // Update tracking history
    cv::Point obstaclePos =
        (obstacleDistance > 0)
            ? cv::Point(frameWidth_ / 2, frameHeight_ - obstacleDistance)
            : cv::Point(-1, -1);
    updateObstacleTracking(obstacleDistance, obstaclePos);

    // Calculate obstacle speed from history

    if (!obstacleDetected_)
    {
        return -1; // Full speed - no obstacle
    }
    else
    {
        obstacleSpeed_ = calculateObstacleSpeed();

        float obstacleDistanceMeters =
            static_cast<float>(currentObstacleDistance_) *
            (farDistance_ - nearDistance_) / frameHeight_;

        // Define safety distances in meters
        const float CRITICAL_DISTANCE_M = 0.1f;
        const float SAFE_DISTANCE_M     = 0.3f;
        const float COMFORT_DISTANCE_M  = 0.5f;

        // Calculate absolute speed of obstacle (in m/s)
        float obstacleAbsoluteSpeed =
            currentSpeed_ + obstacleSpeed_; // obstacleSpeed_ is relative

        // Make sure obstacle speed is not negative (stationary at minimum)
        obstacleAbsoluteSpeed = std::max(0.0f, obstacleAbsoluteSpeed);

        if (obstacleDistanceMeters < CRITICAL_DISTANCE_M)
        {
            // Emergency stop
            std::cout << "EMERGENCY: Obstacle at " << obstacleDistanceMeters
                      << "m - STOPPING" << std::endl;
            return 0.0f;
        }
        else if (obstacleDistanceMeters < SAFE_DISTANCE_M)
        {
            // Close following: match obstacle speed but reduce slightly for
            // safety
            float targetSpeed =
                obstacleAbsoluteSpeed * 0.8f; // 80% of obstacle speed
            std::cout << "CLOSE FOLLOW: Obstacle at " << obstacleDistanceMeters
                      << "m, " << "Obstacle speed: " << obstacleAbsoluteSpeed
                      << "m/s, " << "Target: " << targetSpeed << "m/s"
                      << std::endl;
            return targetSpeed;
        }
        else if (obstacleDistanceMeters < COMFORT_DISTANCE_M)
        {
            // Normal following: match obstacle speed exactly
            std::cout << "FOLLOWING: Obstacle at " << obstacleDistanceMeters
                      << "m, " << "Matching speed: " << obstacleAbsoluteSpeed
                      << "m/s" << std::endl;
            return obstacleAbsoluteSpeed;
        }
        else
        {
            // Far enough: gradually transition to desired cruise speed
            float blendFactor = (obstacleDistanceMeters - COMFORT_DISTANCE_M) /
                                (COMFORT_DISTANCE_M * 0.5f);
            blendFactor = std::clamp(blendFactor, 0.0f, 1.0f);

            float targetSpeed = obstacleAbsoluteSpeed * (1.0f - blendFactor) +
                                desiredSpeed_ * blendFactor;

            std::cout << "TRANSITION: Distance " << obstacleDistanceMeters
                      << "m, " << "Blending to cruise speed: " << targetSpeed
                      << "m/s" << std::endl;
            return targetSpeed;
        }
    }
}

int AdaptiveCruiseControl::findObstacleOnTrajectory(
    const cv::Mat& segmentationMask, const std::vector<cv::Point>& midCurve)
{
    if (segmentationMask.channels() != 3)
    {
        std::cout << "Invalid segmentation mask format" << std::endl;
        return -1;
    }

    // Start from the NEAREST point (bottom/end of trajectory) and work towards
    // the car
    for (int i = static_cast<int>(midCurve.size()) - 1; i >= 0; i--)
    {
        cv::Point trajPoint = midCurve[i];

        // Skip points too close to bottom (ignore zone) and too far ahead
        if (trajPoint.y > frameHeight_ * IGNORE_ZONE_RATIO)
            continue;
        if (trajPoint.y < frameHeight_ * 0.1)
            continue;

        // Calculate trajectory slope at this point
        cv::Vec2f trajectoryDirection =
            calculateTrajectoryDirection(midCurve, i);

        // Create perpendicular vector to trajectory direction for detection
        // line
        cv::Vec2f perpendicular(-trajectoryDirection[1],
                                trajectoryDirection[0]);

        // Normalize perpendicular vector
        float perpLength = std::sqrt(perpendicular[0] * perpendicular[0] +
                                     perpendicular[1] * perpendicular[1]);
        if (perpLength > 0)
        {
            perpendicular[0] /= perpLength;
            perpendicular[1] /= perpLength;
        }

        int nonRoadPixels = 0;
        int totalPixels   = 0;

        // Check along perpendicular line across detection zone
        for (int offset = -DETECTION_ZONE_WIDTH; offset <= DETECTION_ZONE_WIDTH;
             offset++)
        {
            // Calculate point along perpendicular line
            int checkX =
                static_cast<int>(trajPoint.x + offset * perpendicular[0]);
            int checkY =
                static_cast<int>(trajPoint.y + offset * perpendicular[1]);

            if (checkY >= 0 && checkY < frameHeight_ && checkX >= 0 &&
                checkX < frameWidth_)
            {
                cv::Vec3b pixel =
                    segmentationMask.at<cv::Vec3b>(checkY, checkX);
                totalPixels++;

                if (!isRoadPixel(pixel))
                {
                    nonRoadPixels++;
                }
            }
        }

        // If more than 10% of the detection zone is non-road, consider it an
        // obstacle
        if (totalPixels > 0 &&
            (static_cast<float>(nonRoadPixels) / totalPixels) > 0.1f)
        {
            return frameHeight_ - trajPoint.y;
        }
    }

    return -1; // No obstacle found
}

void AdaptiveCruiseControl::updateObstacleTracking(int obstacleDistance,
                                                   const cv::Point& obstaclePos)
{
    double currentTime           = getCurrentTime();
    float obstacleDistanceMeters = static_cast<float>(obstacleDistance) *
                                   (farDistance_ - nearDistance_) /
                                   frameHeight_;

    // Update current state
    previousObstacleDistance_ = currentObstacleDistance_;
    currentObstacleDistance_  = obstacleDistance;
    obstaclePosition_         = obstaclePos;
    obstacleDetected_         = (obstacleDistance > 0);
    std::cout << "OBSTACLE DISTANCE : " << obstacleDistanceMeters << std::endl;
    std::cout << "Time between measures : "
              << currentTime - lastMeasurementTime_ << std::endl;

    // Add to history if obstacle detected
    if (obstacleDetected_)
    {
        ObstacleInfo info;
        info.distance  = obstacleDistanceMeters;
        info.position  = obstaclePos;
        info.timestamp = currentTime;

        obstacleHistory_.push_back(info);

        // Keep only recent history
        while (obstacleHistory_.size() > static_cast<size_t>(MAX_HISTORY))
        {
            obstacleHistory_.pop_front();
        }

        // Remove old entries (older than 2 seconds)
        while (!obstacleHistory_.empty() &&
               (currentTime - obstacleHistory_.front().timestamp) > 2.0)
        {
            obstacleHistory_.pop_front();
        }
    }
    else
    {
        // Clear history if no obstacle detected for a while
        if (currentTime - lastMeasurementTime_ > 0.5)
        {
            obstacleHistory_.clear();
        }
    }

    lastMeasurementTime_ = currentTime;
}

float AdaptiveCruiseControl::calculateObstacleSpeed()
{
    if (obstacleHistory_.size() < 2)
    {
        return 0.0f;
    }

    // Linear regression over the recent history
    double sumTime = 0.0, sumDist = 0.0, sumTimeDist = 0.0,
           sumTimeSquared = 0.0;
    int count             = 0;

    double baseTime = obstacleHistory_.front().timestamp;

    for (const auto& info : obstacleHistory_)
    {
        double t = info.timestamp - baseTime;
        double d = info.distance;

        sumTime += t;
        sumDist += d;
        sumTimeDist += t * d;
        sumTimeSquared += t * t;
        count++;
    }

    if (count < 2 || sumTimeSquared == 0)
        return 0.0f;

    // Calculate slope (speed)
    double slope = (count * sumTimeDist - sumTime * sumDist) /
                   (count * sumTimeSquared - sumTime * sumTime);

    // Relative speed is the slope
    float relativeSpeed = static_cast<float>(slope);

    return relativeSpeed;
}

bool AdaptiveCruiseControl::isRoadPixel(const cv::Vec3b& pixel)
{
    // Check for specific road color in your segmentation
    if (std::abs(pixel[0] - 128) == 0 && std::abs(pixel[1] - 64) == 0 &&
        std::abs(pixel[2] - 128) == 0)
    {
        return true;
    }

    return false; // Non-road pixel
}

cv::Vec2f AdaptiveCruiseControl::calculateTrajectoryDirection(
    const std::vector<cv::Point>& midCurve, int currentIndex)
{
    cv::Vec2f direction(0.0f, 1.0f); // Default to vertical (downward)

    int lookAhead =
        3; // Number of points to look ahead/behind for slope calculation

    // Get previous and next points for slope calculation
    int prevIndex = std::max(0, currentIndex - lookAhead);
    int nextIndex = std::min(static_cast<int>(midCurve.size()) - 1,
                             currentIndex + lookAhead);

    if (prevIndex != nextIndex)
    {
        cv::Point prevPoint = midCurve[prevIndex];
        cv::Point nextPoint = midCurve[nextIndex];

        // Calculate direction vector
        float dx = static_cast<float>(nextPoint.x - prevPoint.x);
        float dy = static_cast<float>(nextPoint.y - prevPoint.y);

        // Normalize direction vector
        float length = std::sqrt(dx * dx + dy * dy);
        if (length > 0)
        {
            direction[0] = dx / length;
            direction[1] = dy / length;
        }
    }

    return direction;
}

double AdaptiveCruiseControl::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}
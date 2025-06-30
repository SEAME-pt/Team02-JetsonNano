#include "TrajectoryDefinition.hpp"
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

TrajectoryDefinition::TrajectoryDefinition(
    std::shared_ptr<zenoh::Session> session, const int height, const int width) : height_(height), width_(width)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    speed_lock_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/Speed/Lock")));
    coeffs_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/Coeffs")));
    ipm_frame_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/Ipm")));
    frame_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/Frame")));
    lane_mask_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/LaneMask")));
    class_mask_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/ObjMask")));

    // cv_stream = cv::cuda::Stream();

    publisher_ = std::make_shared<LaneDetectorPublisher>(session_);

    mpc_trajectory_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/MPC/Trajectory",
        [this](const zenoh::Sample& sample)
        {
            
            std::string traj_str = sample.get_payload().as_string();
            std::vector<cv::Point> points;
            std::stringstream ss(traj_str);
            std::string point_str;
            while (std::getline(ss, point_str, ';')) {
                std::stringstream point_ss(point_str);
                std::string x_str, y_str;
                if (std::getline(point_ss, x_str, ',') && std::getline(point_ss, y_str, ',')) {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    // Convert from MPC coordinates to image coordinates if needed
                    int x_img = static_cast<int>(x + width_/2);      // Undo x shift
                    int y_img = static_cast<int>(height_ - y);    // Undo y flip/shift
                    points.emplace_back(x_img, y_img);
                }
            }
            // Store for use in visualization
            mpcPoints_ = points;
        },
        zenoh::closures::none));

}

TrajectoryDefinition::~TrajectoryDefinition()
{
    if (canBus) {
        delete canBus;
    }
    delete kalmanFilter;
    delete ipm;
    delete avoidance;
}

void TrajectoryDefinition::initLocalEnv() {
    try {
        this->canBus     = new CAN();
        this->canBus->init("/dev/spidev0.0");
    } catch (...) {
        std::cerr << "Error on initializing can" << std::endl;
        this->canBus = NULL;
    }

    try
    {
        this->kalmanFilter = new ::KalmanFilter();
        this->kalmanFilter->init();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing KalmanFilter" << e.what() << std::endl;
    }

    try
    {
        float cameraHeight = 0.137f;       // meters
        float cameraPitch = 20.0f;       // degrees down from horizontal
        float horizontalFOV = 65.1f;     // degrees
        float img_height = static_cast<float>(height_);
        float img_width = static_cast<float>(width_);
        float h_fov_rad = horizontalFOV * CV_PI / 180.0f;
        float verticalFOV = 2.0f * std::atan((img_height/img_width) * std::tan(h_fov_rad/2.0f)) * 180.0f / CV_PI;
        nearDistance_ = 0.01f;       // meters
        farDistance_ = 0.5f;       // meters
        laneWidth_ = 0.4f;      // meters
        cv::Size bevSize = cv::Size(width_, height_);
        cv::Size origSize = cv::Size(width_, height_);

        this->ipm = new IPM();
        this->ipm->init(origSize, bevSize);
        this->ipm->calibrateFromCamera(cameraHeight, cameraPitch, horizontalFOV,
                                       verticalFOV, nearDistance_, farDistance_,
                                       laneWidth_);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing IPM" << e.what() << std::endl;
    }

    try
    {
        this->avoidance = new ObstacleAvoidance(width_, height_, 4);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing ObstacleAvoidance" << e.what() << std::endl;
    }
}

void TrajectoryDefinition::initCarlaEnv() {
    this->canBus = NULL;

    try
    {
        this->kalmanFilter = new ::KalmanFilter();
        this->kalmanFilter->init();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing KalmanFilter" << e.what() << std::endl;
    }

    try
    {
        float cameraHeight = 1.5f;       // meters
        float cameraPitch = 15.0f;       // degrees down from horizontal
        float horizontalFOV = 105.0f;     // degrees
        float img_height = static_cast<float>(height_);
        float img_width = static_cast<float>(width_);
        float h_fov_rad = horizontalFOV * CV_PI / 180.0f;
        float verticalFOV = 2.0f * std::atan((img_height/img_width) * std::tan(h_fov_rad/2.0f)) * 180.0f / CV_PI;
        nearDistance_ = 1.0f;       // meters
        farDistance_ = 10.0f;       // meters
        laneWidth_ = 6.0f;          // meters
        cv::Size bevSize = cv::Size(width_, height_);
        cv::Size origSize = cv::Size(width_, height_);

        this->ipm = new IPM();
        this->ipm->init(origSize, bevSize);
        this->ipm->calibrateFromCamera(cameraHeight, cameraPitch, horizontalFOV,
                                       verticalFOV, nearDistance_, farDistance_,
                                       laneWidth_);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing IPM" << e.what() << std::endl;
    }

    try
    {
        this->avoidance = new ObstacleAvoidance(width_, height_, 8);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing ObstacleAvoidance" << e.what() << std::endl;
    }
}

cv::Mat TrajectoryDefinition::process(cv::Mat& frame, cv::Mat& binary_mask,
                                   cv::Mat& class_mask)
{
    cv::Mat resized_binary_mask;
    cv::resize(binary_mask, resized_binary_mask, frame.size(), 0, 0, cv::INTER_LINEAR);

    cv::Mat resized_class_mask;
    cv::resize(class_mask, resized_class_mask, frame.size(), 0, 0, cv::INTER_LINEAR);

    cv::Mat ipm_binary_mask = ipm->applyIPM(resized_binary_mask);
    cv::Mat ipm_class_mask = ipm->applyIPM(resized_class_mask);
    cv::Mat ipm_frame = ipm->applyIPM(frame);

    createLanes(ipm_frame, ipm_binary_mask, ipm_class_mask);

    cv::Size size(width_ * laneWidth_ / (farDistance_ - nearDistance_), height_);

    cv::Mat res_frame;
    cv::resize(ipm_frame, res_frame, size, 0, 0, cv::INTER_LINEAR);

    cv::Mat res_class_mask;
    cv::resize(ipm_class_mask, res_class_mask, size, 0, 0, cv::INTER_LINEAR);

    cv::addWeighted(res_frame, 0.7, res_class_mask, 0.3, 0.0, res_frame);
    return (res_frame);
}

void TrajectoryDefinition::createLanes(cv::Mat& frame, cv::Mat& binary_mask,
                                       cv::Mat& class_mask)
{
    std::vector<cv::Point> leftCurve;
    std::vector<cv::Point> rightCurve;
    std::vector<cv::Point> midCurve;
    std::vector<std::vector<cv::Point>> lanePolylines;

    currentFrame++;
    allPolylinesViz_ = frame.clone();
    frameWidth_      = frame.cols;
    frameHeight_     = frame.rows;

    lanePolylines = clusterLaneMask(binary_mask, frameWidth_ * frameHeight_ / 3100, frameWidth_ * frameHeight_ / 3100, 6);
    
    drawPolyLanes(lanePolylines);
    // float maxHorizontalDistance = frameWidth_ * 0.10;  // 15% of frame width
    // float maxVerticalGap        = frameHeight_ * 0.20; // 20% of frame height
    // mergeLaneComponents(lanePolylines, maxHorizontalDistance, maxVerticalGap);
    
    filterFalseLanes(lanePolylines);

    if (lanePolylines.size() >= 2) {
        defineLaneEnv(lanePolylines, leftCurve, rightCurve);
    } else if (lanePolylines.size() == 1) {
        if (checkIfLeftLane(lanePolylines[0])) {
            leftCurve = lanePolylines[0];
            onePolyline(leftCurve, rightCurve);
        } else {
            rightCurve = lanePolylines[0];
            onePolyline(leftCurve, rightCurve);
        }
    } else {
        leftCurve  = kalmanFilter->predictLeftLaneCurve(frameHeight_, frameWidth_);
        rightCurve = kalmanFilter->predictRightLaneCurve(frameHeight_, frameWidth_);
    }

    defineTrajectoryCurve(midCurve, leftCurve, rightCurve);
    
    drawCurves(midCurve, leftCurve, rightCurve);
    (void) class_mask;
    // // obstacleAvoidance(class_mask, midCurve);
   
    createMidPointError(midCurve);
    
    // // checkForwardCollision(class_mask, midCurve);

    // mpcDebug();

    allPolylinesViz_.copyTo(frame);
}

void TrajectoryDefinition::filterFalseLanes(std::vector<std::vector<cv::Point>> &lanePolylines) {
    for (int i = static_cast<int>(lanePolylines.size()) - 1; i >= 0; i--)
    {
        if (lanePolylines[i].size() < static_cast<unsigned int>(frameHeight_ * frameWidth_ / 3500))
            lanePolylines.erase(lanePolylines.begin() + i);
        else
            defineLanePolyline(lanePolylines[i]);
    }
}

void TrajectoryDefinition::defineLaneEnv(std::vector<std::vector<cv::Point>> &lanePolylines, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve) {
    if (!prevRightCurve.empty() && !prevLeftCurve.empty()) {
        int bestLeftIdx = -1, bestRightIdx = -1;
        float minLeftDist = FLT_MAX, minRightDist = FLT_MAX;

        for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++) {
            float leftDistance = prevLeftCurve.empty() ? FLT_MAX : calculateLaneDistance(prevLeftCurve, lanePolylines[i]);
            float rightDistance = prevRightCurve.empty() ? FLT_MAX : calculateLaneDistance(prevRightCurve, lanePolylines[i]);

            float threshold = calculateHistoricalLaneWidth() * 0.60;

            if (leftDistance < rightDistance && leftDistance < threshold) {
                if (leftDistance < minLeftDist) {
                    minLeftDist = leftDistance;
                    bestLeftIdx = i;
                }
            } else if (rightDistance < leftDistance && rightDistance < threshold) {
                if (rightDistance < minRightDist) {
                    minRightDist = rightDistance;
                    bestRightIdx = i;
                }
            }
        }

        if (bestLeftIdx != -1 && bestRightIdx != -1 && bestLeftIdx != bestRightIdx) {
            leftCurve = lanePolylines[bestLeftIdx];
            rightCurve = lanePolylines[bestRightIdx];
            
            updateLaneWidthHistory(leftCurve, rightCurve);

            if (leftCurve.size() > 90)
                kalmanFilter->updateLeftLaneFilter(leftCurve);

            if (rightCurve.size() > 90)
                kalmanFilter->updateRightLaneFilter(rightCurve);

            prevLeftCurve = leftCurve;
            prevRightCurve = rightCurve;
            
            leftLaneLastUpdatedFrame = currentFrame;
            rightLaneLastUpdatedFrame = currentFrame;
        } else if (bestLeftIdx != -1) {
            leftCurve = lanePolylines[bestLeftIdx];
            
            onePolyline(leftCurve, rightCurve);
        } else if (bestRightIdx != -1) {
            rightCurve = lanePolylines[bestRightIdx];

            onePolyline(leftCurve, rightCurve);
        } else {
            lowerPointLaneDefinition(lanePolylines, leftCurve, rightCurve);
        }
    } else if (!prevLeftCurve.empty()) {
        int bestLeftIdx = -1;
        float minLeftDist = FLT_MAX;

        for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++) {
            float leftDistance = prevLeftCurve.empty() ? FLT_MAX : calculateLaneDistance(prevLeftCurve, lanePolylines[i]);

            float threshold = calculateHistoricalLaneWidth() * 0.60;

            if (leftDistance < minLeftDist && leftDistance < threshold) {
                minLeftDist = leftDistance;
                bestLeftIdx = i;
            }
        }

        if (bestLeftIdx != -1) {
            leftCurve = lanePolylines[bestLeftIdx];

            float maxAllowedDistance =  calculateHistoricalLaneWidth() * 0.60;

            float leftAvgX = 0.0f;
            for (const auto& pt : leftCurve)
                leftAvgX += pt.x;
            leftAvgX /= leftCurve.size();

            int bestRightIdx = -1;
            float minRightDist = FLT_MAX;

            for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++) {
                float avgX = 0.0f;
                for (const auto& pt : lanePolylines[i])
                    avgX += pt.x;
                avgX /= lanePolylines[i].size();

                // Must be to the left of rightCurve
                if (avgX < leftAvgX) {
                    float dist = calculateLaneDistance(lanePolylines[i], rightCurve);
                    if (dist < minRightDist && dist < maxAllowedDistance) {
                        minRightDist = dist;
                        bestRightIdx = i;
                    }
                }
            }

            if (bestRightIdx != -1) {
                leftCurve = lanePolylines[bestRightIdx];

                updateLaneWidthHistory(leftCurve, rightCurve);

                if (leftCurve.size() > 90)
                    kalmanFilter->updateLeftLaneFilter(leftCurve);

                if (rightCurve.size() > 90)
                    kalmanFilter->updateRightLaneFilter(rightCurve);

                prevLeftCurve = leftCurve;
                prevRightCurve = rightCurve;
                
                leftLaneLastUpdatedFrame = currentFrame;
                rightLaneLastUpdatedFrame = currentFrame;
            } else {
                onePolyline(leftCurve, rightCurve);
            }
        } else {
            lowerPointLaneDefinition(lanePolylines, leftCurve, rightCurve);
        }
    } else if (!prevRightCurve.empty()) {
        int bestRightIdx = -1;
        float minRightDist = FLT_MAX;

        for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++) {
            float rightDistance = prevRightCurve.empty() ? FLT_MAX : calculateLaneDistance(prevRightCurve, lanePolylines[i]);

            float threshold = calculateHistoricalLaneWidth() * 0.60;

            if (rightDistance < minRightDist && rightDistance < threshold) {
                minRightDist = rightDistance;
                bestRightIdx = i;
            }
        }

        if (bestRightIdx != -1) {
            rightCurve = lanePolylines[bestRightIdx];

            float maxAllowedDistance =  calculateHistoricalLaneWidth() * 0.60;

            float rightAvgX = 0.0f;
            for (const auto& pt : rightCurve)
                rightAvgX += pt.x;
            rightAvgX /= rightCurve.size();

            int bestLeftIdx = -1;
            float minLeftDist = FLT_MAX;

            for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++) {
                float avgX = 0.0f;
                for (const auto& pt : lanePolylines[i])
                    avgX += pt.x;
                avgX /= lanePolylines[i].size();

                // Must be to the right of leftCurve
                if (avgX > rightAvgX) {
                    float dist = calculateLaneDistance(lanePolylines[i], rightCurve);
                    if (dist < minLeftDist && dist < maxAllowedDistance) {
                        minLeftDist = dist;
                        bestLeftIdx = i;
                    }
                }
            }

            if (bestLeftIdx != -1) {
                leftCurve = lanePolylines[bestLeftIdx];

                updateLaneWidthHistory(leftCurve, rightCurve);

                if (leftCurve.size() > 90)
                    kalmanFilter->updateLeftLaneFilter(leftCurve);

                if (rightCurve.size() > 90)
                    kalmanFilter->updateRightLaneFilter(rightCurve);

                prevLeftCurve = leftCurve;
                prevRightCurve = rightCurve;
                
                leftLaneLastUpdatedFrame = currentFrame;
                rightLaneLastUpdatedFrame = currentFrame;
            } else {
                onePolyline(leftCurve, rightCurve);
            }
        } else {
            lowerPointLaneDefinition(lanePolylines, leftCurve, rightCurve);
        }
    } else {
        lowerPointLaneDefinition(lanePolylines, leftCurve, rightCurve);
    }
}

void TrajectoryDefinition::lowerPointLaneDefinition(std::vector<std::vector<cv::Point>> &lanePolylines, std::vector<cv::Point>& leftCurve,
    std::vector<cv::Point>& rightCurve) {
    int centerX = frameWidth_ / 2;
    float minLeftDist = FLT_MAX;
    float minRightDist = FLT_MAX;
    int leftIdx = -1;
    int rightIdx = -1;

    for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++)
    {
        // Compute average x for this polyline
        float avgX = 0.0f;
        for (const auto& pt : lanePolylines[i])
            avgX += pt.x;
        avgX /= lanePolylines[i].size();

        if (avgX < centerX) {
            float dist = centerX - avgX;
            if (dist < minLeftDist) {
                minLeftDist = dist;
                leftIdx = i;
            }
        } else {
            float dist = avgX - centerX;
            if (dist < minRightDist) {
                minRightDist = dist;
                rightIdx = i;
            }
        }
    }

    if (leftIdx != -1)
        leftCurve = lanePolylines[leftIdx];
    if (rightIdx != -1)
        rightCurve = lanePolylines[rightIdx];

    updateLaneWidthHistory(leftCurve, rightCurve);

    if (leftCurve.size() > 90)
        kalmanFilter->updateLeftLaneFilter(leftCurve);

    if (rightCurve.size() > 90)
        kalmanFilter->updateRightLaneFilter(rightCurve);

    prevLeftCurve = leftCurve;
    prevRightCurve = rightCurve;
    
    leftLaneLastUpdatedFrame = currentFrame;
    rightLaneLastUpdatedFrame = currentFrame;
}

std::vector<std::vector<cv::Point>>
TrajectoryDefinition::clusterLaneMask(const cv::Mat& laneMask, int kernelSize,
                                      int minArea, int maxLanes)
{
    static cv::Mat verticalKernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernelSize, kernelSize * 2));
    static cv::Mat horizontalKernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

    cv::Mat result = laneMask.clone();
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, verticalKernel);
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, horizontalKernel);

    cv::Mat labels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(result, labels, stats,
                                                     centroids, 8, CV_32S);

    std::vector<std::pair<int, float>> validComponents;
    validComponents.reserve(std::min(numLabels, maxLanes + 3));

    for (int i = 1; i < numLabels; i++)
    {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > minArea)
        {
            float centerX = centroids.at<double>(i, 0);
            validComponents.push_back(std::make_pair(i, centerX));
        }
    }

    if (validComponents.size() > static_cast<size_t>(maxLanes))
    {
        std::partial_sort(
            validComponents.begin(), validComponents.begin() + maxLanes,
            validComponents.end(),
            [](const std::pair<int, float>& a, const std::pair<int, float>& b)
            { return a.second < b.second; });
        validComponents.resize(maxLanes);
    }
    else
    {
        std::sort(
            validComponents.begin(), validComponents.end(),
            [](const std::pair<int, float>& a, const std::pair<int, float>& b)
            { return a.second < b.second; });
    }

    std::vector<std::vector<cv::Point>> lanePolylines;
    lanePolylines.reserve(validComponents.size());

    for (const auto& comp : validComponents)
    {
        int compIdx = comp.first;

        std::vector<cv::Point> lanePoints;
        lanePoints.reserve(labels.rows / 5);

        for (int y = 0; y < labels.rows; y += 2)
        {
            const int* row = labels.ptr<int>(y);
            int xStart = -1, xEnd = -1;

            for (int x = 0; x < labels.cols; x++)
            {
                if (row[x] == compIdx)
                {
                    if (xStart < 0)
                        xStart = x;
                    xEnd = x;
                }
            }

            if (xStart >= 0)
            {
                int midX = (xStart + xEnd) / 2;
                lanePoints.push_back(cv::Point(midX, y));
            }
        }

        if (!lanePoints.empty())
        {
            lanePolylines.push_back(std::move(lanePoints));
        }
    }

    return lanePolylines;
}

void TrajectoryDefinition::mergeLaneComponents(
    std::vector<std::vector<cv::Point>>& lanePolylines, float maxHorizontalDist,
    float maxVerticalGap)
{
    if (lanePolylines.size() <= 1)
        return;

    bool mergePerformed = true;
     while (mergePerformed)
    {
        mergePerformed = false;

        for (size_t i = 0; i < lanePolylines.size() && !mergePerformed; i++)
        {
            for (size_t j = i + 1; j < lanePolylines.size() && !mergePerformed; j++)
            {
                float minDistance = FLT_MAX;
                cv::Point closestPt1, closestPt2;
                
                for (const auto& pt1 : lanePolylines[i]) {
                    for (const auto& pt2 : lanePolylines[j]) {
                        float dist = cv::norm(pt1 - pt2);
                        if (dist < minDistance) {
                            minDistance = dist;
                            closestPt1 = pt1;
                            closestPt2 = pt2;
                        }
                    }
                }
                
                float avgX1 = 0, avgX2 = 0;
                int minY1 = INT_MAX, maxY1 = 0;
                int minY2 = INT_MAX, maxY2 = 0;

                for (const auto& pt : lanePolylines[i])
                {
                    minY1 = std::min(minY1, pt.y);
                    maxY1 = std::max(maxY1, pt.y);
                    avgX1 += pt.x;
                }
                avgX1 /= lanePolylines[i].size();

                for (const auto& pt : lanePolylines[j])
                {
                    minY2 = std::min(minY2, pt.y);
                    maxY2 = std::max(maxY2, pt.y);
                    avgX2 += pt.x;
                }
                avgX2 /= lanePolylines[j].size();
                
                float directDistance = minDistance;
                
                bool similarDirection = true;
                if (!lanePolylines[i].empty() && lanePolylines[i].size() > 1 &&
                    !lanePolylines[j].empty() && lanePolylines[j].size() > 1)
                {
                    cv::Point dir1 = lanePolylines[i].back() - lanePolylines[i].front();

                    cv::Point dir2 = lanePolylines[j].back() - lanePolylines[j].front();

                    float dotProduct = dir1.x * dir2.x + dir1.y * dir2.y;
                    similarDirection = (dotProduct > 0); // Positive dot product means similar direction
                }
                
                float combinedThreshold = std::sqrt(maxHorizontalDist * maxHorizontalDist + 
                                                  maxVerticalGap * maxVerticalGap);

                if (directDistance <= combinedThreshold && similarDirection)
                {
                    lanePolylines[i].insert(lanePolylines[i].end(),
                                           lanePolylines[j].begin(),
                                           lanePolylines[j].end());
                    lanePolylines.erase(lanePolylines.begin() + j);
                    mergePerformed = true;
                    break;
                }
            }
        }
    }
}

void TrajectoryDefinition::onePolyline(std::vector<cv::Point>& leftCurve,
    std::vector<cv::Point>& rightCurve) {
    if (rightCurve.empty())
    {
        rightCurve = kalmanFilter->predictRightLaneCurve(frameHeight_, frameWidth_);
        checkPredicedCurve(rightCurve, leftCurve, true);
        
        if (leftCurve.size() > 90)
            kalmanFilter->updateLeftLaneFilter(leftCurve);

        prevLeftCurve = leftCurve;
        prevRightCurve.clear();

        leftLaneLastUpdatedFrame = currentFrame;

        cv::putText(allPolylinesViz_, "Left (valid)", cv::Point(10, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 3);
        cv::putText(allPolylinesViz_, "Right (predicted)", cv::Point(20, 160),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 3);
    }
    else
    {
        leftCurve = kalmanFilter->predictLeftLaneCurve(frameHeight_, frameWidth_);
        checkPredicedCurve(leftCurve, rightCurve, false);
        
        if (rightCurve.size() > 90)
            kalmanFilter->updateRightLaneFilter(rightCurve);

        prevLeftCurve.clear();
        prevRightCurve = rightCurve;

        rightLaneLastUpdatedFrame = currentFrame;

        cv::putText(allPolylinesViz_, "Right (valid)", cv::Point(10, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 3);
        cv::putText(allPolylinesViz_, "Left (predicted)", cv::Point(20, 160),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 3);
    }
}

float TrajectoryDefinition::calculateLaneDistance(
    const std::vector<cv::Point>& lane1, const std::vector<cv::Point>& lane2)
{
    std::map<int, cv::Point> lane1Points;
    std::map<int, cv::Point> lane2Points;

    // Normalize Y values to 0-100 range
    for (const auto& pt : lane1)
    {
        int normY          = (pt.y * 100) / frameHeight_; // Assuming height_ is max height
        lane1Points[normY] = pt;
    }

    for (const auto& pt : lane2)
    {
        int normY          = (pt.y * 100) / frameHeight_; // Assuming height_ is max height
        lane2Points[normY] = pt;
    }

    float totalDist = 0;
    int matchCount  = 0;

    for (const auto& p1 : lane1Points)
    {
        int y = p1.first;
        if (lane2Points.find(y) != lane2Points.end())
        {
            float dist = cv::norm(p1.second - lane2Points[y]);
            totalDist += dist;
            matchCount++;
        }
    }

    return (matchCount > 0) ? (totalDist / matchCount) : FLT_MAX;
}

float TrajectoryDefinition::calculateHistoricalLaneWidth() {    
    if (recentWidths.empty()) {
        return frameWidth_ * 0.50;
    } else {
        float sum = 0.0f;
        for (const float& width : recentWidths) {
            sum += width;
        }
        return sum / recentWidths.size();
    }
}

void TrajectoryDefinition::updateLaneWidthHistory(const std::vector<cv::Point>& leftLane, 
                                                const std::vector<cv::Point>& rightLane) {    
    float avgDistance = calculateLaneDistance(leftLane, rightLane);
    
    if (avgDistance > frameWidth_ * 0.10f && avgDistance < frameWidth_ * 0.90f) {
        recentWidths.push_back(avgDistance);
        
        if (recentWidths.size() > static_cast<unsigned int>(MAX_WIDTH_HISTORY)) {
            recentWidths.pop_front();
        }
    }
}

void TrajectoryDefinition::checkPredicedCurve(
    std::vector<cv::Point>& predictedCurve,
    const std::vector<cv::Point>& realLane, bool isLeftLane)
{
    float avgMiddleX        = 0;
    float avgDetectedX      = 0;
    float expectedWidth = calculateHistoricalLaneWidth();
    float expectedMiddleX   = 0;

    // Calculate average X positions
    for (const auto& pt : predictedCurve)
    {
        avgMiddleX += pt.x;
    }
    avgMiddleX /= predictedCurve.size();

    for (const auto& pt : realLane)
    {
        avgDetectedX += pt.x;
    }
    avgDetectedX /= realLane.size();

    if (isLeftLane)
    {
        expectedMiddleX = avgDetectedX + expectedWidth;
    }
    else
    {
        expectedMiddleX = avgDetectedX - expectedWidth;
    }

    float error = std::abs(avgMiddleX - expectedMiddleX);

    if (error > frameWidth_ * 0.15f)
    {
        cv::putText(allPolylinesViz_, "Invalid curve prediction - using offset",
                    cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 255, 0), 1);
        std::cout << "Invalid curve prediction - using offset" << std::endl;

        predictedCurve.clear();
        predictedCurve.reserve(realLane.size());

        float divergence = 0.1f;

        if (isLeftLane)
        {
            // for (const auto& pt : realLane)
            // {
            //     predictedCurve.push_back(
            //         cv::Point(pt.x + expectedWidth, pt.y));
            // }
            for (const auto& pt : realLane)
            {
                // The farther down (higher y), the more we shift to the right
                int extra_offset = static_cast<int>(divergence * (pt.y - realLane.front().y));
                predictedCurve.push_back(
                    cv::Point(pt.x + expectedWidth + extra_offset, pt.y));
            }
            kalmanFilter->updateLeftLaneFilter(predictedCurve);
        }
        else
        {
            // for (const auto& pt : realLane)
            // {
            //     predictedCurve.push_back(
            //         cv::Point(pt.x - expectedWidth, pt.y));
            // }
            for (const auto& pt : realLane)
            {
                // The farther down (higher y), the more we shift to the left
                int extra_offset = static_cast<int>(divergence * (pt.y - realLane.front().y));
                predictedCurve.push_back(
                    cv::Point(pt.x - expectedWidth - extra_offset, pt.y));
            }
            kalmanFilter->updateRightLaneFilter(predictedCurve);
        }

        defineLanePolyline(predictedCurve);
    }
}

void TrajectoryDefinition::defineTrajectoryCurve(
    std::vector<cv::Point>& midCurve, std::vector<cv::Point>& leftCurve,
    std::vector<cv::Point>& rightCurve)
{
    int numPoints = std::min(leftCurve.size(), rightCurve.size());
    for (int i = 0; i < numPoints; i++)
    {
        size_t leftIdx  = i * leftCurve.size() / numPoints;
        size_t rightIdx = i * rightCurve.size() / numPoints;

        int midX = (leftCurve[leftIdx].x + rightCurve[rightIdx].x) / 2;
        int midY = (leftCurve[leftIdx].y + rightCurve[rightIdx].y) / 2;
        midCurve.push_back(cv::Point(midX, midY));

    }
    
    publishCoeffs(midCurve);
}

void TrajectoryDefinition::drawCurves(
    std::vector<cv::Point>& midCurve, std::vector<cv::Point>& leftCurve,
    std::vector<cv::Point>& rightCurve)
{
    cv::Scalar leftCurveColor = cv::Scalar(255, 0, 0);
    for (size_t i = 1; i < leftCurve.size(); i++)
    {
        cv::line(allPolylinesViz_, leftCurve[i - 1], leftCurve[i], leftCurveColor,
                 8);
    }

    cv::Scalar midCurveColor = cv::Scalar(255, 255, 255);
    for (size_t i = 1; i < midCurve.size(); i++)
    {
        cv::line(allPolylinesViz_, midCurve[i - 1], midCurve[i], midCurveColor,
                 8);
    }

    cv::Scalar rightCurveColor = cv::Scalar(0, 0, 255);
    for (size_t i = 1; i < rightCurve.size(); i++)
    {
        cv::line(allPolylinesViz_, rightCurve[i - 1], rightCurve[i], rightCurveColor,
                 8);
    }
}


void TrajectoryDefinition::drawPolyLanes(
    std::vector<std::vector<cv::Point>> lanePolylines)
{
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),   // Blue
        cv::Scalar(0, 255, 0),   // Green
        cv::Scalar(255, 255, 0), // Cyan
        cv::Scalar(255, 0, 255), // Magenta
        cv::Scalar(0, 255, 255),  // Yellow
        cv::Scalar(0, 0, 255)   // Red
    };

    // Draw each polyline with a different color
    for (size_t i = 0; i < lanePolylines.size(); i++)
    {
        cv::Scalar color = colors[i % colors.size()];
        for (size_t j = 1; j < lanePolylines[i].size(); j++)
        {
            cv::line(allPolylinesViz_, lanePolylines[i][j - 1],
                     lanePolylines[i][j], color, 3);
        }
    }
}

void TrajectoryDefinition::defineLanePolyline(
    std::vector<cv::Point>& curve) 
{
    std::sort(curve.begin(), curve.end(),
              [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });

    std::vector<double> x_values, y_values;
    for (const auto& pt : curve)
    {
        y_values.push_back(static_cast<double>(pt.y));
        x_values.push_back(static_cast<double>(pt.x));
    }

    cv::Mat coeffs;
    if (x_values.size() >= 150)
    {
        cv::Mat y_mat(y_values), x_mat;
        x_mat.create(y_values.size(), 4, CV_64F);
        for (int i = 0; i < x_mat.rows; i++)
        {
            x_mat.at<double>(i, 0) = 1.0;
            x_mat.at<double>(i, 1) = y_values[i];
            x_mat.at<double>(i, 2) = y_values[i] * y_values[i];
            x_mat.at<double>(i, 3) = y_values[i] * y_values[i] * y_values[i];
        }

        cv::solve(x_mat, cv::Mat(x_values), coeffs, cv::DECOMP_SVD);
        
        curve.clear();

        int numSamples =
            frameHeight_ / 100;
        for (int y = 0; y < frameHeight_; y += numSamples)
        {
            double yVal = static_cast<double>(y);
            double xVal = coeffs.at<double>(0) + coeffs.at<double>(1) * yVal +
                          coeffs.at<double>(2) * yVal * yVal +
                          coeffs.at<double>(3) * yVal * yVal * yVal;
            curve.push_back(cv::Point(static_cast<int>(xVal), y));
        }
    }
    else if (x_values.size() >= 4)
    {
        // 2nd-degree polynomial fit
        cv::Mat y_mat(y_values), x_mat;
        x_mat.create(y_values.size(), 3, CV_64F);
        for (int i = 0; i < x_mat.rows; i++)
        {
            x_mat.at<double>(i, 0) = 1.0;
            x_mat.at<double>(i, 1) = y_values[i];
            x_mat.at<double>(i, 2) = y_values[i] * y_values[i];
        }
        cv::solve(x_mat, cv::Mat(x_values), coeffs, cv::DECOMP_SVD);

        curve.clear();
        int numSamples = frameHeight_ / 100;
        for (int y = 0; y < frameHeight_; y += numSamples)
        {
            double yVal = static_cast<double>(y);
            double xVal = coeffs.at<double>(0) + coeffs.at<double>(1) * yVal +
                          coeffs.at<double>(2) * yVal * yVal;
            curve.push_back(cv::Point(static_cast<int>(xVal), y));
        }
    }
    else
    {
        std::cerr << "Not enough points to calculate coefficients" << std::endl;
    }
}

void TrajectoryDefinition::createMidPointError(std::vector<cv::Point>& midCurve)
{
    cv::Point midPoint;

    if (!midCurve.empty())
    {
        int targetY = frameHeight_ - (1.5 * frameHeight_ / 3); // 3/6 up from bottom for local
        // int targetY = frameHeight_ - (1.0 * frameHeight_ / 3); // 1/3 up from bottom for carla

        // Find closest point to target Y
        size_t closestIdx = 0;
        int minDistance   = std::abs(midCurve[0].y - targetY);

        for (size_t i = 1; i < midCurve.size(); i++)
        {
            int distance = std::abs(midCurve[i].y - targetY);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIdx  = i;
            }
        }

        // Use the point at found index
        midPoint = midCurve[closestIdx];

        cv::circle(allPolylinesViz_, midPoint, 8, cv::Scalar(255, 0, 255), -1);

        float centerX  = frameWidth_ / 2;
        float rawError = (midPoint.x - centerX) / (frameWidth_ / 2.0f);

        // Apply rate limiting to error changes
        static float prevError       = 0.0f;
        const float MAX_ERROR_CHANGE = 1.0f; // Maximum allowed change per frame

        float errorChange = rawError - prevError;
        if (std::abs(errorChange) > MAX_ERROR_CHANGE)
        {
            errorChange =
                (errorChange > 0) ? MAX_ERROR_CHANGE : -MAX_ERROR_CHANGE;
        }

        float lateralError = prevError + errorChange;
        prevError          = lateralError;

        const float MAX_ERROR = 3.0f;
        if (lateralError > MAX_ERROR)
        {
            lateralError = MAX_ERROR;
            prevError    = MAX_ERROR; // Update prevError as well
        }
        else if (lateralError < -MAX_ERROR)
        {
            lateralError = -MAX_ERROR;
            prevError    = -MAX_ERROR; // Update prevError as well
        }

        publisher_->publishCameraError(lateralError);

        std::string statusMsg =
            "Error: " + std::to_string(lateralError).substr(0, 6);
        cv::putText(allPolylinesViz_, statusMsg, cv::Point(60, 100),
                    cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 2);
    }
}

bool TrajectoryDefinition::checkIfLeftLane(
    const std::vector<cv::Point>& lanePolyline)
{
    cv::Point lowestPoint(-1, -1);
    int centerX = frameWidth_ / 2;
    float avgX  = 0;

     // Find lowest point and average X position
    for (const auto& pt : lanePolyline)
    {
        if (pt.y > lowestPoint.y)
        {
            lowestPoint = pt;
        }
        avgX += pt.x;
    }
    avgX /= lanePolyline.size();

    bool hasValidLeftMemory =
        (currentFrame - leftLaneLastUpdatedFrame) < MAX_LANE_MEMORY_FRAMES;
    bool hasValidRightMemory =
        (currentFrame - rightLaneLastUpdatedFrame) < MAX_LANE_MEMORY_FRAMES;
    bool isLeftLane;

    // If we have previous lanes, use them to identify current lane
    if (hasValidLeftMemory || hasValidRightMemory)
    {
        float leftDistance =
            hasValidLeftMemory
                ? calculateLaneDistance(lanePolyline, prevLeftCurve)
                : FLT_MAX;

        float rightDistance =
            hasValidRightMemory
                ? calculateLaneDistance(lanePolyline, prevRightCurve)
                : FLT_MAX;

        if (hasValidLeftMemory)
        {
            float leftStaleness =
                1.0f + 0.05f * (currentFrame - leftLaneLastUpdatedFrame);
            leftDistance *= leftStaleness;
        }

        if (hasValidRightMemory)
        {
            float rightStaleness =
                1.0f + 0.05f * (currentFrame - rightLaneLastUpdatedFrame);
            rightDistance *= rightStaleness;
        }

        isLeftLane = leftDistance < rightDistance;

        std::string debugMsg =
            "Memory match: " + std::string(isLeftLane ? "LEFT" : "RIGHT") +
            " (L:" + std::to_string(leftDistance).substr(0, 5) +
            "/R:" + std::to_string(rightDistance).substr(0, 5) + ")";
        cv::putText(allPolylinesViz_, debugMsg, cv::Point(20, 80),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

        std::string staleMsg =
            "Staleness - L:" +
            std::to_string(currentFrame - leftLaneLastUpdatedFrame) +
            " R:" + std::to_string(currentFrame - rightLaneLastUpdatedFrame);
        cv::putText(allPolylinesViz_, staleMsg, cv::Point(20, 100),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    }
    else
    {
        // Fallback to position-based detection
        isLeftLane = avgX < centerX;

        if (!hasValidLeftMemory || !hasValidRightMemory)
        {
            cv::putText(allPolylinesViz_, "Memory expired - using position",
                        cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 0, 255), 2);
        }
        else
        {
            cv::putText(allPolylinesViz_, "Position-based detection",
                        cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 0), 2);
        }
    }

    return (isLeftLane);
}

bool TrajectoryDefinition::checkForwardCollision(
    const cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve)
{
    if (midCurve.empty())
    {
        // Draw a message when no trajectory is available
        cv::putText(allPolylinesViz_,
                    "No trajectory available for collision check",
                    cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 255), 2);
        return false;
    }

    // Check if segmentation mask is valid
    if (segmentation_mask.empty() || segmentation_mask.channels() != 3) {
        cv::putText(allPolylinesViz_, "Invalid segmentation mask format: " + 
                    std::to_string(segmentation_mask.channels()) + " channels",
                    cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 255), 2);
        return false;
    }

    const int zoneWidth = frameWidth_ * 0.20; // Width of zone around trajectory
    int total_pixels    = 0;
    int road_pixels     = 0;

    std::vector<cv::Point> rightPoints;
    std::vector<cv::Point> polygonPoints;

    for (size_t i = 0; i < midCurve.size(); i++)
    {
        if (midCurve[i].y < frameHeight_ * 0.25)
            continue;

        cv::Point direction;
        if (i > 0)
        {
            direction = midCurve[i] - midCurve[i - 1];
        }
        else if (i < midCurve.size() - 1)
        {
            direction = midCurve[i + 1] - midCurve[i];
        }
        else
        {
            direction = cv::Point(0, 1); // Default vertical
        }

        float length =
            std::sqrt(direction.x * direction.x + direction.y * direction.y);
        if (length > 0)
        {
            direction.x = direction.x / length;
            direction.y = direction.y / length;
        }

        // Create perpendicular vector for width
        cv::Point perpendicular(-direction.y, direction.x);

        // Left and right boundary points
        polygonPoints.push_back(
            cv::Point(midCurve[i].x - perpendicular.x * zoneWidth / 2,
                      midCurve[i].y - perpendicular.y * zoneWidth / 2));

        // Store right points separately to create a complete polygon later
        rightPoints.push_back(
            cv::Point(midCurve[i].x + perpendicular.x * zoneWidth / 2,
                      midCurve[i].y + perpendicular.y * zoneWidth / 2));
    }

    if (polygonPoints.size() < 3 || rightPoints.empty())
    {
        // Not enough points to create a polygon
        cv::putText(allPolylinesViz_,
                    "Insufficient trajectory points for collision check",
                    cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 255), 2);
        return true;
    }


    // Add right points in reverse order to complete the polygon
    for (int i = rightPoints.size() - 1; i >= 0; i--)
    {
        polygonPoints.push_back(rightPoints[i]);
    }

    // Create a mask for the polygon
    cv::Mat polygonMask = cv::Mat::zeros(segmentation_mask.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours = {polygonPoints};

    if (polygonPoints.empty())
    {
        cv::putText(allPolylinesViz_,
                    "Empty polygon - cannot check for collision",
                    cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 255), 2);
        return true;
    }

    // After creating the polygon mask
    cv::fillPoly(polygonMask, contours, cv::Scalar(255));

    // Count road pixels in the polygon
    for (int y = 0; y < polygonMask.rows && y < segmentation_mask.rows; y++) {
        const uchar* maskRow = polygonMask.ptr<uchar>(y);
        const cv::Vec3b* segRow = segmentation_mask.ptr<cv::Vec3b>(y);
        
        for (int x = 0; x < polygonMask.cols && x < segmentation_mask.cols; x++) {
            // Only check pixels inside the polygon
            if (maskRow[x] > 0) {
                total_pixels++;
                
                // Check road pixel using safer comparison
                cv::Vec3b pixel = segRow[x];
                
                // Use a more tolerant color comparison
                if ((std::abs(pixel[0] - 128) < 10 && 
                    std::abs(pixel[1] - 64) < 10 && 
                    std::abs(pixel[2] - 128) < 10) || 
                    (pixel[0] < 10 && pixel[1] < 10 && pixel[2] < 10)) {
                    road_pixels++;
                }
            }
        }
    }

    // Draw the polygon for visualization
    cv::polylines(allPolylinesViz_, contours, true, cv::Scalar(0, 255, 255), 2);

    // Calculate road percentage and check for danger
    float road_percentage = static_cast<float>(road_pixels) /
                            (total_pixels + 1); // Avoid division by zero
    const float SAFE_ROAD_THRESHOLD = 0.7;      // 70% of zone should be road
    bool danger_detected            = (road_percentage < SAFE_ROAD_THRESHOLD);

    // // Display road percentage
    // std::string roadText =
    //     "Road: " + std::to_string(int(road_percentage * 100)) + "%" +
    //     " Total: " + std::to_string(total_pixels);
    // cv::putText(allPolylinesViz_, roadText, cv::Point(20, 120),
    //             cv::FONT_HERSHEY_SIMPLEX, 0.5,
    //             danger_detected ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0),
    //             2);

    if (danger_detected)
    {
        cv::putText(allPolylinesViz_, "OBSTACLE DETECTED!",
                    cv::Point(frameWidth_ / 2 - 150, frameHeight_ / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 3);

        std::cout << "\033[1;31m*** WARNING: OBSTACLE DETECTED! STOPPING "
                     "VEHICLE ***\033[0m"
                  << std::endl;

        is_emergency_stop = true;

        publishSpeedLock("1");
        if (this->canBus) {
            try
            {
                uint8_t value[8];
                memcpy(value, "DANGER", sizeof(value));

            this->canBus->writeMessage(0x200, value, sizeof(value));
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error sending CAN message on Object Detector: "
                        << e.what() << std::endl;
            }
        }
        return true;
        }
    else if (is_emergency_stop)
    {
        std::cout << "\033[1;32m*** PATH CLEAR - READY TO RESUME ***\033[0m"
                << std::endl;
        is_emergency_stop = false;

        publishSpeedLock("0");
    }
    return false;
}

void TrajectoryDefinition::publishSpeedLock(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    speed_lock_publisher_->put(std::move(buf));
}

void TrajectoryDefinition::publishCoeffs(std::vector<cv::Point>& curve)
{
    std::sort(curve.begin(), curve.end(),
              [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });

    std::vector<double> x_values, y_values;

    cv::Mat coeffs;
    for (const auto& pt : curve)
    {
        double y_prime = height_ - static_cast<double>(pt.y);   // Flip and shift y
        double x_prime = static_cast<double>(pt.x) - width_/2;   // Shift x
        y_values.push_back(y_prime);
        x_values.push_back(x_prime);
    }
    
    if (x_values.size() >= 4)
    {
        cv::Mat y_mat(y_values), x_mat;

        // Create Vandermonde matrix for polynomial fitting

        x_mat.create(y_values.size(), 4, CV_64F);
        for (int i = 0; i < x_mat.rows; i++)
        {
            double y_prime = y_values[i];
            x_mat.at<double>(i, 0) = 1.0;
            x_mat.at<double>(i, 1) = y_prime;
            x_mat.at<double>(i, 2) = y_prime * y_prime;
            x_mat.at<double>(i, 3) = y_prime * y_prime * y_prime;
        }
        cv::solve(x_mat, cv::Mat(x_values), coeffs, cv::DECOMP_SVD);
        std::ostringstream oss;
        for (int i = 0; i < coeffs.rows; ++i)
        {
            oss << coeffs.at<double>(i);
            if (i < coeffs.rows - 1)
                oss << ",";
        }
        std::string coeffs_str = oss.str();
        // std::cout << "Coefficients: " << coeffs_str << std::endl;

        const auto len = coeffs_str.size() + 1;
        auto alloc_result =
            provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
        zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
        memcpy(buf.data(), coeffs_str.c_str(), len);
        coeffs_publisher_->put(std::move(buf));
    }
    else
    {
        std::cerr << "Not enough points to calculate coefficients" << std::endl;
        return;
    }
}

void TrajectoryDefinition::obstacleAvoidance(cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve)
{
    try {
        avoidance->buildOccupancy(segmentation_mask);
        avoidance->buildTrajectoryGrid(midCurve);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error in obstacle avoidance lalala: " << e.what() << std::endl;
    }
    
    try {
        if (avoidance->detectAllCollisions())
        {
            std::vector<cv::Point> adjustedTrajectory = avoidance->adjustTrajectory(midCurve);        
            // Replace original trajectory with adjusted one
            midCurve.clear();
            midCurve = adjustedTrajectory;

            cv::Scalar midCurveColor = cv::Scalar(0, 255, 0); // White
            for (size_t i = 1; i < midCurve.size(); i++)
            {
                cv::line(allPolylinesViz_, midCurve[i - 1], midCurve[i], midCurveColor,
                        3);
            }
        }
        // avoidance->visualizeGrid(&midCurve, segmentation_mask);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error in obstacle avoidance: " << e.what() << std::endl;
    }
}

void TrajectoryDefinition::mpcDebug(void) {
    // Draw the predicted trajectory as a green polyline
    if (mpcPoints_.size() > 1) {
        for (size_t i = 1; i < mpcPoints_.size(); ++i) {
            std::cout << "Drawing MPC point: " << mpcPoints_[i] << std::endl;
            cv::line(allPolylinesViz_, mpcPoints_[i - 1], mpcPoints_[i], cv::Scalar(0, 255, 0), 2);
        }
    }
}

void TrajectoryDefinition::publishIPMFrame(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    ipm_frame_publisher_->put(std::move(buf));
}

void TrajectoryDefinition::publishOrigFrame(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    frame_publisher_->put(std::move(buf));
}

void TrajectoryDefinition::publishBinMask(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    lane_mask_publisher_->put(std::move(buf));
}

void TrajectoryDefinition::publishClassMask(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    class_mask_publisher_->put(std::move(buf));
}
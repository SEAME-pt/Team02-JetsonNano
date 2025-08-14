#include "TrajectoryDefinition.hpp"
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

TrajectoryDefinition::TrajectoryDefinition(
    std::shared_ptr<zenoh::Session> session, const int height, const int width)
    : height_(height), width_(width)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    emergency_brake_publisher_.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/Speed/Emergency")));
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
    lkas_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/ADAS/LKAS")));
    sae_2_disable_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/ADAS/SAE_2")));
    autonomy_env_enable_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/ADAS/Enable")));
    acc_speed_publisher_.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/ADAS/acc_speed")));

    publisher_ = std::make_shared<LaneDetectorPublisher>(session_);

    activeAutonomyLevel_subscriber_.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/ActiveAutonomyLevel",
        [this](const zenoh::Sample& sample)
        {
            std::string activeAutonomyLevel = sample.get_payload().as_string();

            if (activeAutonomyLevel.find("SAE_0") != std::string::npos)
                activeAutonomyLevel_ = "SAE_0";
            else if (activeAutonomyLevel.find("SAE_1_LKAS") !=
                     std::string::npos)
                activeAutonomyLevel_ = "SAE_1_LKAS";
            else if (activeAutonomyLevel.find("SAE_1_ACC") != std::string::npos)
                activeAutonomyLevel_ = "SAE_1_ACC";
            else if (activeAutonomyLevel.find("SAE_2") != std::string::npos)
                activeAutonomyLevel_ = "SAE_2";
            else if (activeAutonomyLevel.find("SAE_3") != std::string::npos)
                activeAutonomyLevel_ = "SAE_3";
            else if (activeAutonomyLevel.find("SAE_4") != std::string::npos)
                activeAutonomyLevel_ = "SAE_4";
        },
        zenoh::closures::none));

    mpc_trajectory_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/MPC/Trajectory",
        [this](const zenoh::Sample& sample)
        {
            std::string traj_str = sample.get_payload().as_string();
            std::vector<cv::Point> points;
            std::stringstream ss(traj_str);
            std::string point_str;
            while (std::getline(ss, point_str, ';'))
            {
                std::stringstream point_ss(point_str);
                std::string x_str, y_str;
                if (std::getline(point_ss, x_str, ',') &&
                    std::getline(point_ss, y_str, ','))
                {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    // Convert from MPC coordinates to image coordinates if
                    // needed
                    int x_img =
                        static_cast<int>(x + width_ / 2); // Undo x shift
                    int y_img =
                        static_cast<int>(height_ - y); // Undo y flip/shift
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
    delete kalmanFilter;
    delete ipm;
    delete avoidance;
    delete accontroller;
}

void TrajectoryDefinition::initLocalEnv()
{
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
        float cameraHeight  = 0.146f; // meters
        float cameraPitch   = 20.0f;  // degrees down from horizontal
        float horizontalFOV = 65.1f;  // degrees
        float img_height    = static_cast<float>(height_);
        float img_width     = static_cast<float>(width_);
        float h_fov_rad     = horizontalFOV * CV_PI / 180.0f;
        float verticalFOV =
            2.0f *
            std::atan((img_height / img_width) * std::tan(h_fov_rad / 2.0f)) *
            180.0f / CV_PI;
        nearDistance_     = 0.2f; // meters
        nearDistance_     = 0.2f; // meters
        farDistance_      = 0.8f; // meters
        laneWidth_        = 0.6f; // meters
        cv::Size bevSize  = cv::Size(width_, height_);
        cv::Size origSize = cv::Size(width_, height_);

        this->ipm = new IPM();
        this->ipm->init(origSize, bevSize);
        this->ipm->calibrateFromCamera(cameraHeight, cameraPitch, horizontalFOV,
                                       verticalFOV, nearDistance_, farDistance_,
                                       laneWidth_);

        // For ACC usage
        distanceToObstacle_ = img_height;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing IPM" << e.what() << std::endl;
    }

    try
    {
        this->avoidance    = new ObstacleAvoidance(width_, height_, 8);
        this->accontroller = new AdaptiveCruiseControl(
            session_, width_, height_, nearDistance_, farDistance_, laneWidth_);
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Error initializing ObstacleAvoidance and AdaptiveCruiseControl"
            << e.what() << std::endl;
    }
}

void TrajectoryDefinition::initCarlaEnv()
{
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
        float cameraHeight  = 1.5f;   // meters
        float cameraPitch   = 15.0f;  // degrees down from horizontal
        float horizontalFOV = 105.0f; // degrees
        float img_height    = static_cast<float>(height_);
        float img_width     = static_cast<float>(width_);
        float h_fov_rad     = horizontalFOV * CV_PI / 180.0f;
        float verticalFOV =
            2.0f *
            std::atan((img_height / img_width) * std::tan(h_fov_rad / 2.0f)) *
            180.0f / CV_PI;
        nearDistance_     = 1.0f;  // meters
        farDistance_      = 10.0f; // meters
        laneWidth_        = 6.0f;  // meters
        cv::Size bevSize  = cv::Size(width_, height_);
        cv::Size origSize = cv::Size(width_, height_);

        this->ipm = new IPM();
        this->ipm->init(origSize, bevSize);
        this->ipm->calibrateFromCamera(cameraHeight, cameraPitch, horizontalFOV,
                                       verticalFOV, nearDistance_, farDistance_,
                                       laneWidth_);

        // For ACC usage
        distanceToObstacle_ = img_height;
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
        std::cerr << "Error initializing ObstacleAvoidance" << e.what()
                  << std::endl;
    }
}

cv::Mat TrajectoryDefinition::process(cv::Mat& frame, cv::Mat& binary_mask,
                                      cv::Mat& class_mask)
{
    cv::Mat resized_binary_mask;
    cv::resize(binary_mask, resized_binary_mask, frame.size(), 0, 0,
               cv::INTER_LINEAR);

    cv::Mat resized_class_mask;
    cv::resize(class_mask, resized_class_mask, frame.size(), 0, 0,
               cv::INTER_LINEAR);

    cv::Mat ipm_binary_mask = ipm->applyIPM(resized_binary_mask);
    cv::Mat ipm_class_mask  = ipm->applyIPM(resized_class_mask);
    cv::Mat ipm_frame       = ipm->applyIPM(frame);

    createLanes(ipm_frame, ipm_binary_mask, ipm_class_mask);

    cv::Size size(width_ * laneWidth_ / (farDistance_ - nearDistance_),
                  height_);

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
    std::vector<cv::Mat> coeffsSave;

    currentFrame++;
    allPolylinesViz_ = frame.clone();
    frameWidth_      = frame.cols;
    frameHeight_     = frame.rows;

    lanePolylines = clusterLaneMask(binary_mask, 2, 30, 6);
    clusterObjMask(class_mask, 50);

    filterFalseLanes(lanePolylines, coeffsSave);

    drawPolyLanes(lanePolylines);

    if (lanePolylines.size() >= 2)
    {
        defineLaneEnv(lanePolylines, leftCurve, rightCurve, coeffsSave);
        defineTrajectoryCurve(midCurve, leftCurve, rightCurve);
        checkAutonomyEnvEnable(midCurve);
    }
    else if (lanePolylines.size() == 1)
    {
        if (checkIfLeftLane(lanePolylines[0]))
        {
            leftCurve = lanePolylines[0];
            onePolyline(leftCurve, rightCurve);
        }
        else
        {
            rightCurve = lanePolylines[0];
            onePolyline(leftCurve, rightCurve);
        }
        defineTrajectoryCurve(midCurve, leftCurve, rightCurve);
        publishAutonomyEnvEnable("false");
    }
    else
    {
        leftCurve =
            kalmanFilter->predictLeftLaneCurve(frameHeight_, frameWidth_);
        rightCurve =
            kalmanFilter->predictRightLaneCurve(frameHeight_, frameWidth_);

        defineTrajectoryCurve(midCurve, leftCurve, rightCurve);
        publishAutonomyEnvEnable("false");

        leftCurve.clear();
        rightCurve.clear();
    }

    drawCurves(midCurve, leftCurve, rightCurve);

    if (activeAutonomyLevel_ == "SAE_0")
    {
        prevLeftCurve.clear();
        prevRightCurve.clear();
        recentWidths.clear();
    }
    else
    {
        checkForwardCollision(class_mask, midCurve);
    }

    if (activeAutonomyLevel_ == "SAE_2" || activeAutonomyLevel_ == "SAE_3" ||
        activeAutonomyLevel_ == "SAE_4")
    {
        createMidPointError(midCurve);
        publishCoeffs(midCurve);
        if (activeAutonomyLevel_ == "SAE_4")
        {
            obstacleAvoidance(class_mask, midCurve);
        }
        else if (activeAutonomyLevel_ == "SAE_2")
        {
            adaptiveSpeedControl(class_mask, midCurve);
        }
    }
    else if (activeAutonomyLevel_ == "SAE_1_ACC")
    {
        std::cout << "ACC Mode" << std::endl;
        adaptiveSpeedControl(class_mask, midCurve);
    }

    mpcDebug();

    allPolylinesViz_.copyTo(frame);
}

void TrajectoryDefinition::filterFalseLanes(
    std::vector<std::vector<cv::Point>>& lanePolylines,
    std::vector<cv::Mat>& coeffsSave)
{
    for (int i = static_cast<int>(lanePolylines.size()) - 1; i >= 0; i--)
    {
        if (lanePolylines[i].size() <
            static_cast<unsigned int>(frameHeight_ * frameWidth_ / 6000))
            lanePolylines.erase(lanePolylines.begin() + i);
        else
            coeffsSave.push_back(defineLanePolyline(lanePolylines[i]));
    }
}

void TrajectoryDefinition::defineLaneEnv(
    std::vector<std::vector<cv::Point>>& lanePolylines,
    std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve,
    std::vector<cv::Mat>& coeffsSave)
{
    if (!prevRightCurve.empty() && !prevLeftCurve.empty())
    {
        int bestLeftIdx = -1, bestRightIdx = -1;
        float minLeftDist = FLT_MAX, minRightDist = FLT_MAX;

        for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++)
        {
            float leftDistance =
                calculateLaneDistance(prevLeftCurve, lanePolylines[i]);
            float rightDistance =
                calculateLaneDistance(prevRightCurve, lanePolylines[i]);

            float maxDistance =
                calculateHistoricalLaneWidth() * distance_percentage;

            if (leftDistance < rightDistance && leftDistance < maxDistance)
            {
                if (leftDistance < minLeftDist)
                {
                    minLeftDist = leftDistance;
                    bestLeftIdx = i;
                }
            }
            else if (rightDistance < leftDistance &&
                     rightDistance < maxDistance)
            {
                if (rightDistance < minRightDist)
                {
                    minRightDist = rightDistance;
                    bestRightIdx = i;
                }
            }
        }

        if (bestLeftIdx != -1 && bestRightIdx != -1 &&
            bestLeftIdx != bestRightIdx)
        {
            leftCurve  = lanePolylines[bestLeftIdx];
            rightCurve = lanePolylines[bestRightIdx];

            updateLaneWidthHistory(leftCurve, rightCurve);

            kalmanFilter->updateLeftLaneFilter(leftCurve);
            kalmanFilter->updateRightLaneFilter(rightCurve);

            prevLeftCurve  = leftCurve;
            prevRightCurve = rightCurve;

            leftLaneLastUpdatedFrame  = currentFrame;
            rightLaneLastUpdatedFrame = currentFrame;

            checkLKASEnvEnable(leftCurve, rightCurve, coeffsSave[bestLeftIdx],
                               coeffsSave[bestRightIdx]);
        }
        else if (bestLeftIdx != -1)
        {
            leftCurve = lanePolylines[bestLeftIdx];

            onePolyline(leftCurve, rightCurve);
        }
        else if (bestRightIdx != -1)
        {
            rightCurve = lanePolylines[bestRightIdx];

            onePolyline(leftCurve, rightCurve);
        }
        else
        {
            lowerPointLaneDefinition(lanePolylines, leftCurve, rightCurve);
            std::cout
                << "Lower Point Lane Definition in prev history for both lanes"
                << std::endl;
        }
    }
    else if (!prevLeftCurve.empty())
    {
        int bestLeftIdx   = -1;
        float minLeftDist = FLT_MAX;

        for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++)
        {
            float leftDistance =
                calculateLaneDistance(prevLeftCurve, lanePolylines[i]);

            float maxDistance =
                calculateHistoricalLaneWidth() * distance_percentage;

            if (leftDistance < minLeftDist && leftDistance < maxDistance)
            {
                minLeftDist = leftDistance;
                bestLeftIdx = i;
            }
        }

        if (bestLeftIdx != -1)
        {
            leftCurve = lanePolylines[bestLeftIdx];

            float maxDistance = calculateHistoricalLaneWidth() * 1.20;
            float minDistance = calculateHistoricalLaneWidth() * 0.80;

            float leftAvgX = 0.0f;
            for (const auto& pt : leftCurve)
                leftAvgX += pt.x;
            leftAvgX /= leftCurve.size();

            int bestRightIdx   = -1;
            float minRightDist = FLT_MAX;

            for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++)
            {
                if (i == bestLeftIdx)
                {
                    continue;
                }
                float avgX = 0.0f;
                for (const auto& pt : lanePolylines[i])
                    avgX += pt.x;
                avgX /= lanePolylines[i].size();

                // Must be to the right of leftCurve
                if (avgX > leftAvgX)
                {
                    float dist =
                        calculateLaneDistance(lanePolylines[i], leftCurve);
                    if (dist < minRightDist && dist < maxDistance &&
                        dist > minDistance)
                    {
                        minRightDist = dist;
                        bestRightIdx = i;
                    }
                }
            }

            if (bestRightIdx != -1)
            {
                rightCurve = lanePolylines[bestRightIdx];

                updateLaneWidthHistory(leftCurve, rightCurve);

                kalmanFilter->updateLeftLaneFilter(leftCurve);
                kalmanFilter->updateRightLaneFilter(rightCurve);

                prevLeftCurve  = leftCurve;
                prevRightCurve = rightCurve;

                leftLaneLastUpdatedFrame  = currentFrame;
                rightLaneLastUpdatedFrame = currentFrame;
            }
            else
            {
                onePolyline(leftCurve, rightCurve);
            }
        }
        else
        {
            lowerPointLaneDefinition(lanePolylines, leftCurve, rightCurve);
        }
    }
    else if (!prevRightCurve.empty())
    {
        int bestRightIdx   = -1;
        float minRightDist = FLT_MAX;

        for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++)
        {
            float rightDistance =
                calculateLaneDistance(prevRightCurve, lanePolylines[i]);

            float maxDistance =
                calculateHistoricalLaneWidth() * distance_percentage;

            if (rightDistance < minRightDist && rightDistance < maxDistance)
            {
                minRightDist = rightDistance;
                bestRightIdx = i;
            }
        }

        if (bestRightIdx != -1)
        {
            rightCurve = lanePolylines[bestRightIdx];

            float maxDistance = calculateHistoricalLaneWidth() * 1.20;
            float minDistance = calculateHistoricalLaneWidth() * 0.80;

            float rightAvgX = 0.0f;
            for (const auto& pt : rightCurve)
                rightAvgX += pt.x;
            rightAvgX /= rightCurve.size();

            int bestLeftIdx   = -1;
            float minLeftDist = FLT_MAX;

            for (int i = 0; i < static_cast<int>(lanePolylines.size()); i++)
            {
                if (i == bestRightIdx)
                {
                    continue;
                }

                float avgX = 0.0f;
                for (const auto& pt : lanePolylines[i])
                    avgX += pt.x;
                avgX /= lanePolylines[i].size();

                if (avgX < rightAvgX)
                {
                    float dist =
                        calculateLaneDistance(lanePolylines[i], rightCurve);
                    if (dist < minLeftDist && dist < maxDistance &&
                        dist > minDistance)
                    {
                        minLeftDist = dist;
                        bestLeftIdx = i;
                    }
                }
            }

            if (bestLeftIdx != -1)
            {
                leftCurve = lanePolylines[bestLeftIdx];

                updateLaneWidthHistory(leftCurve, rightCurve);

                kalmanFilter->updateLeftLaneFilter(leftCurve);
                kalmanFilter->updateRightLaneFilter(rightCurve);

                prevLeftCurve  = leftCurve;
                prevRightCurve = rightCurve;

                leftLaneLastUpdatedFrame  = currentFrame;
                rightLaneLastUpdatedFrame = currentFrame;
            }
            else
            {
                onePolyline(leftCurve, rightCurve);
            }
        }
        else
        {
            lowerPointLaneDefinition(lanePolylines, leftCurve, rightCurve);
        }
    }
    else
    {
        lowerPointLaneDefinition(lanePolylines, leftCurve, rightCurve);
    }
}

void TrajectoryDefinition::lowerPointLaneDefinition(
    std::vector<std::vector<cv::Point>>& lanePolylines,
    std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve)
{
    int centerX = frameWidth_ / 2;
    int leftIdx = -1, rightIdx = -1;
    int minLeftDist = INT_MAX, minRightDist = INT_MAX;

    std::vector<int> lowestXs;
    std::vector<int> lowestYs;

    // Find lowest point for each polyline
    for (const auto& poly : lanePolylines)
    {
        int lowestY = -1;
        int lowestX = -1;
        for (const auto& pt : poly)
        {
            if (pt.y > lowestY)
            {
                lowestY = pt.y;
                lowestX = pt.x;
            }
        }
        lowestXs.push_back(lowestX);
        lowestYs.push_back(lowestY);
    }

    // Find closest to center from left and right
    for (size_t i = 0; i < lowestXs.size(); ++i)
    {
        int x = lowestXs[i];
        if (x < centerX)
        {
            int dist = centerX - x;
            if (dist < minLeftDist)
            {
                minLeftDist = dist;
                leftIdx     = i;
            }
        }
        else
        {
            int dist = x - centerX;
            if (dist < minRightDist)
            {
                minRightDist = dist;
                rightIdx     = i;
            }
        }
    }

    if (leftIdx != -1)
        leftCurve = lanePolylines[leftIdx];
    if (rightIdx != -1)
        rightCurve = lanePolylines[rightIdx];

    updateLaneWidthHistory(leftCurve, rightCurve);

    kalmanFilter->updateLeftLaneFilter(leftCurve);
    kalmanFilter->updateRightLaneFilter(rightCurve);

    prevLeftCurve  = leftCurve;
    prevRightCurve = rightCurve;

    leftLaneLastUpdatedFrame  = currentFrame;
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

    int roi_top                                      = laneMask.rows / 4;
    laneMask(cv::Rect(0, 0, laneMask.cols, roi_top)) = 0;

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

void TrajectoryDefinition::clusterObjMask(const cv::Mat& classMask,
                                          int kernelSize)
{
    static cv::Mat verticalKernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernelSize, kernelSize * 2));
    static cv::Mat horizontalKernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

    cv::morphologyEx(classMask, classMask, cv::MORPH_CLOSE, verticalKernel);
    cv::morphologyEx(classMask, classMask, cv::MORPH_CLOSE, horizontalKernel);
}

void TrajectoryDefinition::onePolyline(std::vector<cv::Point>& leftCurve,
                                       std::vector<cv::Point>& rightCurve)
{
    if (rightCurve.empty())
    {
        predictCurve(rightCurve, leftCurve, true);

        kalmanFilter->updateLeftLaneFilter(leftCurve);

        prevLeftCurve = leftCurve;
        prevRightCurve.clear();

        leftLaneLastUpdatedFrame = currentFrame;

        std::cout << "Left (Valid) | Right (Predicted)" << std::endl;
    }
    else
    {
        predictCurve(leftCurve, rightCurve, false);

        kalmanFilter->updateRightLaneFilter(rightCurve);

        prevLeftCurve.clear();
        prevRightCurve = rightCurve;

        rightLaneLastUpdatedFrame = currentFrame;

        std::cout << "Right (Valid) | Left (Predicted)" << std::endl;
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
        int normY =
            (pt.y * 100) / frameHeight_; // Assuming height_ is max height
        lane1Points[normY] = pt;
    }

    for (const auto& pt : lane2)
    {
        int normY =
            (pt.y * 100) / frameHeight_; // Assuming height_ is max height
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

float TrajectoryDefinition::calculateHistoricalLaneWidth()
{
    if (recentWidths.empty())
    {
        return frameWidth_ * 0.30;
    }
    else
    {
        float sum = 0.0f;
        for (const float& width : recentWidths)
        {
            sum += width;
        }
        return sum / recentWidths.size();
    }
}

void TrajectoryDefinition::updateLaneWidthHistory(
    const std::vector<cv::Point>& leftLane,
    const std::vector<cv::Point>& rightLane)
{
    float avgDistance = calculateLaneDistance(leftLane, rightLane);

    // std::cout << "AVG Distance: " << avgDistance << std::endl;
    // std::cout << "Min Distance: " << frameWidth_ * 0.20 << std::endl;
    // std::cout << "Max Distance: " << frameWidth_ * 0.35 << std::endl;

    if (avgDistance > frameWidth_ * 0.20 && avgDistance < frameWidth_ * 0.35)
    {
        recentWidths.push_back(avgDistance);

        if (recentWidths.size() > static_cast<unsigned int>(MAX_WIDTH_HISTORY))
        {
            recentWidths.pop_front();
        }
    }
}

void TrajectoryDefinition::predictCurve(std::vector<cv::Point>& predictedCurve,
                                        const std::vector<cv::Point>& realLane,
                                        bool isLeftLane)
{
    float expectedWidth = calculateHistoricalLaneWidth();

    predictedCurve.reserve(realLane.size());

    std::vector<cv::Point> sortedRealLane = realLane;
    std::sort(sortedRealLane.begin(), sortedRealLane.end(),
              [](const cv::Point& a, const cv::Point& b) { return a.y > b.y; });

    for (size_t i = 0; i < sortedRealLane.size(); ++i)
    {
        cv::Point2f pt = sortedRealLane[i];

        cv::Point2f dir;
        if (i == 0)
            dir = cv::Point2f(sortedRealLane[i + 1]) - pt;
        else if (i == sortedRealLane.size() - 1)
            dir = pt - cv::Point2f(sortedRealLane[i - 1]);
        else
            dir = cv::Point2f(sortedRealLane[i + 1]) -
                  cv::Point2f(sortedRealLane[i - 1]);

        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (len > 1e-3)
            dir /= len;
        else
            dir = cv::Point2f(0, 1); // Default vertical

        cv::Point2f normal1(-dir.y, dir.x);
        cv::Point2f normal2(dir.y, -dir.x);

        float offset           = expectedWidth;
        cv::Point2f candidate1 = pt + normal1 * offset;
        cv::Point2f candidate2 = pt + normal2 * offset;

        cv::Point2f newPt;
        if (isLeftLane)
        {
            newPt = (candidate1.x > pt.x) ? candidate1 : candidate2;
        }
        else
        {
            newPt = (candidate1.x < pt.x) ? candidate1 : candidate2;
        }

        predictedCurve.push_back(
            cv::Point(static_cast<int>(newPt.x), static_cast<int>(newPt.y)));
    }
    defineLanePolyline(predictedCurve);
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
}

void TrajectoryDefinition::drawCurves(std::vector<cv::Point>& midCurve,
                                      std::vector<cv::Point>& leftCurve,
                                      std::vector<cv::Point>& rightCurve)
{
    cv::Scalar leftCurveColor = cv::Scalar(255, 0, 0);
    for (size_t i = 1; i < leftCurve.size(); i++)
    {
        cv::line(allPolylinesViz_, leftCurve[i - 1], leftCurve[i],
                 leftCurveColor, 8);
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
        cv::line(allPolylinesViz_, rightCurve[i - 1], rightCurve[i],
                 rightCurveColor, 8);
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
        cv::Scalar(0, 255, 255), // Yellow
        cv::Scalar(0, 0, 255)    // Red
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

cv::Mat TrajectoryDefinition::defineLanePolyline(std::vector<cv::Point>& curve)
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

        int numSamples = frameHeight_ / 100;
        for (int y = 0; y < frameHeight_; y += numSamples)
        {
            double yVal = static_cast<double>(y);
            double xVal = coeffs.at<double>(0) + coeffs.at<double>(1) * yVal +
                          coeffs.at<double>(2) * yVal * yVal +
                          coeffs.at<double>(3) * yVal * yVal * yVal;
            curve.push_back(cv::Point(static_cast<int>(xVal), y));
        }

        return coeffs;
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

        return coeffs;
    }
    else
    {
        std::cerr << "Not enough points to calculate coefficients" << std::endl;
        return coeffs;
    }
}

void TrajectoryDefinition::createMidPointError(std::vector<cv::Point>& midCurve)
{
    cv::Point midPoint;

    if (!midCurve.empty())
    {
        // int targetY = frameHeight_ - (1.5 * frameHeight_ / 3); // 3/6 up from
        // bottom
        int targetY =
            frameHeight_ - (0.7 * frameHeight_ / 3); // 1/3 up from bottom

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
        // cv::putText(allPolylinesViz_, debugMsg, cv::Point(20, 80),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0),
        //             2);

        std::cout << debugMsg << std::endl;

        std::string staleMsg =
            "Staleness - L:" +
            std::to_string(currentFrame - leftLaneLastUpdatedFrame) +
            " R:" + std::to_string(currentFrame - rightLaneLastUpdatedFrame);
        // cv::putText(allPolylinesViz_, staleMsg, cv::Point(20, 100),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0),
        //             1);

        std::cout << staleMsg << std::endl;
    }
    else
    {
        // Fallback to position-based detection
        isLeftLane = avgX < centerX;

        if (!hasValidLeftMemory || !hasValidRightMemory)
        {
            // cv::putText(allPolylinesViz_, "Memory expired - using position",
            //             cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6,
            //             cv::Scalar(0, 0, 255), 2);
        }
        else
        {
            // cv::putText(allPolylinesViz_, "Position-based detection",
            //             cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6,
            //             cv::Scalar(255, 255, 0), 2);
        }
    }

    return (isLeftLane);
}

bool TrajectoryDefinition::isCurveStraight(const cv::Mat& coeffs,
                                           double threshold)
{
    if (coeffs.rows >= 4)
    {
        // double c = std::abs(coeffs.at<double>(1));
        double d = std::abs(coeffs.at<double>(2));
        double e = std::abs(coeffs.at<double>(3));
        // std::cout << "c: " << c << " d: " << d << " e: " << e << std::endl;
        return (d < threshold && e < threshold);
    }
    else if (coeffs.rows == 3)
    {
        // double c = std::abs(coeffs.at<double>(1));
        double d = std::abs(coeffs.at<double>(2));
        // std::cout << "c: " << c << " d: " << d << std::endl;
        return (d < threshold);
    }

    return true;
}

void TrajectoryDefinition::checkLKASEnvEnable(
    std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve,
    cv::Mat& leftCoeffs, cv::Mat& rightCoeffs)
{
    if (isCurveStraight(rightCoeffs, 1e-2) && isCurveStraight(leftCoeffs, 1e-2))
    {
        float centerX   = frameWidth_ / 2;
        float laneWidth = calculateHistoricalLaneWidth();

        float avgXLeftCurve = 0.0f;
        for (const auto& pt : leftCurve)
            avgXLeftCurve += pt.x;
        avgXLeftCurve /= leftCurve.size();

        float avgXRightCurve = 0.0f;
        for (const auto& pt : rightCurve)
            avgXRightCurve += pt.x;
        avgXRightCurve /= rightCurve.size();

        float diff = (centerX - avgXLeftCurve) / laneWidth;

        publishLKAS(std::to_string(diff));
    }
}

void TrajectoryDefinition::checkAutonomyEnvEnable(
    std::vector<cv::Point>& midCurve)
{
    float avgX      = 0.0f;
    float centerX   = frameWidth_ / 2;
    float laneWidth = calculateHistoricalLaneWidth();

    for (const auto& pt : midCurve)
        avgX += pt.x;
    avgX /= midCurve.size();

    float diff = (centerX - avgX) / laneWidth;

    if (diff < 0.35 && diff > -0.35)
    {
        publishAutonomyEnvEnable("true");
    }
    else
    {
        publishAutonomyEnvEnable("false");
    }
}

// bool TrajectoryDefinition::checkForwardCollision(
//     const cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve)
// {
//     if (midCurve.empty())
//     {
//         // Draw a message when no trajectory is available
//         cv::putText(allPolylinesViz_,
//                     "No trajectory available for collision check",
//                     cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
//                     cv::Scalar(0, 0, 255), 2);
//         return false;
//     }

//     // Check if segmentation mask is valid
//     if (segmentation_mask.empty() || segmentation_mask.channels() != 3)
//     {
//         cv::putText(allPolylinesViz_,
//                     "Invalid segmentation mask format: " +
//                         std::to_string(segmentation_mask.channels()) +
//                         " channels",
//                     cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
//                     cv::Scalar(0, 0, 255), 2);
//         return false;
//     }

//     const int zoneWidth = frameWidth_ * 0.20; // Width of zone around trajectory
//     int total_pixels    = 0;
//     int road_pixels     = 0;

//     std::vector<cv::Point> rightPoints;
//     std::vector<cv::Point> polygonPoints;

//     for (size_t i = 0; i < midCurve.size(); i++)
//     {
//         if (midCurve[i].y < frameHeight_ * 0.25)
//             continue;

//         cv::Point direction;
//         if (i > 0)
//         {
//             direction = midCurve[i] - midCurve[i - 1];
//         }
//         else if (i < midCurve.size() - 1)
//         {
//             direction = midCurve[i + 1] - midCurve[i];
//         }
//         else
//         {
//             direction = cv::Point(0, 1); // Default vertical
//         }

//         float length =
//             std::sqrt(direction.x * direction.x + direction.y * direction.y);
//         if (length > 0)
//         {
//             direction.x = direction.x / length;
//             direction.y = direction.y / length;
//         }

//         // Create perpendicular vector for width
//         cv::Point perpendicular(-direction.y, direction.x);

//         // Left and right boundary points
//         polygonPoints.push_back(
//             cv::Point(midCurve[i].x - perpendicular.x * zoneWidth / 2,
//                       midCurve[i].y - perpendicular.y * zoneWidth / 2));

//         // Store right points separately to create a complete polygon later
//         rightPoints.push_back(
//             cv::Point(midCurve[i].x + perpendicular.x * zoneWidth / 2,
//                       midCurve[i].y + perpendicular.y * zoneWidth / 2));
//     }

//     if (polygonPoints.size() < 3 || rightPoints.empty())
//     {
//         // Not enough points to create a polygon
//         cv::putText(allPolylinesViz_,
//                     "Insufficient trajectory points for collision check",
//                     cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
//                     cv::Scalar(0, 0, 255), 2);
//         return true;
//     }

//     // Add right points in reverse order to complete the polygon
//     for (int i = rightPoints.size() - 1; i >= 0; i--)
//     {
//         polygonPoints.push_back(rightPoints[i]);
//     }

//     // Create a mask for the polygon
//     cv::Mat polygonMask = cv::Mat::zeros(segmentation_mask.size(), CV_8UC1);
//     std::vector<std::vector<cv::Point>> contours = {polygonPoints};

//     if (polygonPoints.empty())
//     {
//         cv::putText(allPolylinesViz_,
//                     "Empty polygon - cannot check for collision",
//                     cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
//                     cv::Scalar(0, 0, 255), 2);
//         return true;
//     }

//     // After creating the polygon mask
//     cv::fillPoly(polygonMask, contours, cv::Scalar(255));

//     // Count road pixels in the polygon
//     for (int y = 0; y < polygonMask.rows && y < segmentation_mask.rows; y++)
//     {
//         const uchar* maskRow    = polygonMask.ptr<uchar>(y);
//         const cv::Vec3b* segRow = segmentation_mask.ptr<cv::Vec3b>(y);

//         for (int x = 0; x < polygonMask.cols && x < segmentation_mask.cols; x++)
//         {
//             // Only check pixels inside the polygon
//             if (maskRow[x] > 0)
//             {
//                 total_pixels++;

//                 // Check road pixel using safer comparison
//                 cv::Vec3b pixel = segRow[x];

//                 // Use a more tolerant color comparison
//                 if ((std::abs(pixel[0] - 128) < 10 &&
//                      std::abs(pixel[1] - 64) < 10 &&
//                      std::abs(pixel[2] - 128) < 10) ||
//                     (pixel[0] < 10 && pixel[1] < 10 && pixel[2] < 10))
//                 {
//                     road_pixels++;
//                 }
//             }
//         }
//     }

//     // Draw the polygon for visualization
//     cv::polylines(allPolylinesViz_, contours, true, cv::Scalar(0, 255, 255), 2);

//     // Calculate road percentage and check for danger
//     float road_percentage = static_cast<float>(road_pixels) /
//                             (total_pixels + 1); // Avoid division by zero
//     const float SAFE_ROAD_THRESHOLD = 0.3;      // 30% of zone should be road
//     bool danger_detected            = (road_percentage < SAFE_ROAD_THRESHOLD);

//     if (danger_detected)
//     {
//         cv::putText(allPolylinesViz_, "OBSTACLE DETECTED!",
//                     cv::Point(frameWidth_ / 2 - 150, frameHeight_ / 2),
//                     cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 3);

//         is_emergency_stop = true;

//         publishEmergencyBrake("1");

//         return true;
//     }
//     else if (is_emergency_stop)
//     {
//         // std::cout << "\033[1;32m*** PATH CLEAR - READY TO RESUME ***\033[0m"
//         //         << std::endl;
//         is_emergency_stop = false;

//         publishEmergencyBrake("0");
//     }
//     return false;
// }

bool TrajectoryDefinition::checkForwardCollision(
    const cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve)
{
    if (midCurve.empty())
    {
        cv::putText(allPolylinesViz_,
                    "No trajectory available for collision check",
                    cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 255), 2);
        return false;
    }

    if (segmentation_mask.empty() || segmentation_mask.channels() != 3)
    {
        cv::putText(allPolylinesViz_,
                    "Invalid segmentation mask format: " +
                        std::to_string(segmentation_mask.channels()) +
                        " channels",
                    cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 255), 2);
        return false;
    }

    const float zoneWidthRatio = 0.25f; // Adjustable width ratio (25% of frame width)
    const int zoneWidth = frameWidth_ * zoneWidthRatio;
    int total_pixels = 0;
    int road_pixels = 0;

    std::vector<cv::Point> leftBoundary;
    std::vector<cv::Point> rightBoundary;
    std::vector<cv::Point> polygonPoints;

    // Create smooth boundaries that follow trajectory curvature
    for (size_t i = 0; i < midCurve.size(); i++)
    {
        // Skip points too close to top of frame
        if (midCurve[i].y < frameHeight_ * 0.25)
            continue;

        cv::Point2f tangent = calculateSmoothTangent(midCurve, i);
        
        // Create perpendicular vector (normal) to the tangent
        cv::Point2f normal(-tangent.y, tangent.x);
        
        // Calculate boundary points
        cv::Point2f center(midCurve[i].x, midCurve[i].y);
        cv::Point2f leftPoint = center - normal * (zoneWidth / 2.0f);
        cv::Point2f rightPoint = center + normal * (zoneWidth / 2.0f);
        
        // Ensure points are within frame bounds
        leftPoint.x = std::max(0.0f, std::min(static_cast<float>(frameWidth_ - 1), leftPoint.x));
        leftPoint.y = std::max(0.0f, std::min(static_cast<float>(frameHeight_ - 1), leftPoint.y));
        rightPoint.x = std::max(0.0f, std::min(static_cast<float>(frameWidth_ - 1), rightPoint.x));
        rightPoint.y = std::max(0.0f, std::min(static_cast<float>(frameHeight_ - 1), rightPoint.y));
        
        leftBoundary.push_back(cv::Point(static_cast<int>(leftPoint.x), static_cast<int>(leftPoint.y)));
        rightBoundary.push_back(cv::Point(static_cast<int>(rightPoint.x), static_cast<int>(rightPoint.y)));
    }

    if (leftBoundary.size() < 3 || rightBoundary.size() < 3)
    {
        cv::putText(allPolylinesViz_,
                    "Insufficient trajectory points for collision check",
                    cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 255), 2);
        return true;
    }

    // Create closed polygon: left boundary + reversed right boundary
    polygonPoints = leftBoundary;
    polygonPoints.insert(polygonPoints.end(), rightBoundary.rbegin(), rightBoundary.rend());

    // Apply smoothing to the polygon for better curvature following
    polygonPoints = smoothPolygon(polygonPoints);

    // Create mask and check for obstacles
    cv::Mat polygonMask = cv::Mat::zeros(segmentation_mask.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours = {polygonPoints};
    cv::fillPoly(polygonMask, contours, cv::Scalar(255));

    // Count road pixels in the polygon
    for (int y = 0; y < polygonMask.rows && y < segmentation_mask.rows; y++)
    {
        const uchar* maskRow = polygonMask.ptr<uchar>(y);
        const cv::Vec3b* segRow = segmentation_mask.ptr<cv::Vec3b>(y);

        for (int x = 0; x < polygonMask.cols && x < segmentation_mask.cols; x++)
        {
            if (maskRow[x] > 0)
            {
                total_pixels++;
                cv::Vec3b pixel = segRow[x];

                // Check for road pixels (adjust color thresholds as needed)
                if ((std::abs(pixel[0] - 128) < 15 &&
                     std::abs(pixel[1] - 64) < 15 &&
                     std::abs(pixel[2] - 128) < 15) ||
                    (pixel[0] < 15 && pixel[1] < 15 && pixel[2] < 15))
                {
                    road_pixels++;
                }
            }
        }
    }

    // Visualize the detection zone
    cv::polylines(allPolylinesViz_, contours, true, cv::Scalar(0, 255, 255), 2);
    
    // Optional: Draw the left and right boundaries separately for debugging
    if (leftBoundary.size() > 1)
        cv::polylines(allPolylinesViz_, leftBoundary, false, cv::Scalar(255, 0, 0), 1);
    if (rightBoundary.size() > 1)
        cv::polylines(allPolylinesViz_, rightBoundary, false, cv::Scalar(0, 0, 255), 1);

    // Calculate road percentage and check for danger
    float road_percentage = static_cast<float>(road_pixels) / (total_pixels + 1);
    const float SAFE_ROAD_THRESHOLD = 0.3f;
    bool danger_detected = (road_percentage < SAFE_ROAD_THRESHOLD);

    if (danger_detected)
    {
        cv::putText(allPolylinesViz_, "OBSTACLE DETECTED!",
                    cv::Point(frameWidth_ / 2 - 150, frameHeight_ / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 3);

        is_emergency_stop = true;
        publishEmergencyBrake("1");
        return true;
    }
    else if (is_emergency_stop)
    {
        is_emergency_stop = false;
        publishEmergencyBrake("0");
    }
    
    return false;
}

// Helper function to calculate smooth tangent at a point
cv::Point2f TrajectoryDefinition::calculateSmoothTangent(const std::vector<cv::Point>& curve, size_t index)
{
    cv::Point2f tangent;
    
    if (curve.size() < 2)
    {
        return cv::Point2f(0, 1); // Default vertical direction
    }
    
    if (index == 0)
    {
        // First point: use forward difference
        tangent = cv::Point2f(curve[1] - curve[0]);
    }
    else if (index == curve.size() - 1)
    {
        // Last point: use backward difference
        tangent = cv::Point2f(curve[index] - curve[index - 1]);
    }
    else
    {
        // Middle points: use central difference for smoother tangent
        cv::Point2f forward = cv::Point2f(curve[index + 1] - curve[index]);
        cv::Point2f backward = cv::Point2f(curve[index] - curve[index - 1]);
        tangent = (forward + backward) * 0.5f;
    }
    
    // Normalize the tangent vector
    float length = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
    if (length > 1e-6f)
    {
        tangent.x /= length;
        tangent.y /= length;
    }
    else
    {
        tangent = cv::Point2f(0, 1); // Default vertical if zero length
    }
    
    return tangent;
}

// Helper function to smooth polygon points
std::vector<cv::Point> TrajectoryDefinition::smoothPolygon(const std::vector<cv::Point>& polygon)
{
    if (polygon.size() < 3)
        return polygon;
    
    std::vector<cv::Point> smoothed;
    smoothed.reserve(polygon.size());
    
    const float smoothingFactor = 0.3f; // Adjust for more/less smoothing
    
    for (size_t i = 0; i < polygon.size(); i++)
    {
        size_t prev = (i == 0) ? polygon.size() - 1 : i - 1;
        size_t next = (i == polygon.size() - 1) ? 0 : i + 1;
        
        cv::Point2f current(polygon[i]);
        cv::Point2f prevPt(polygon[prev]);
        cv::Point2f nextPt(polygon[next]);
        
        // Apply simple smoothing filter
        cv::Point2f smoothedPt = current * (1.0f - smoothingFactor) + 
                                (prevPt + nextPt) * (smoothingFactor * 0.5f);
        
        smoothed.push_back(cv::Point(static_cast<int>(smoothedPt.x), static_cast<int>(smoothedPt.y)));
    }
    
    return smoothed;
}

void TrajectoryDefinition::obstacleAvoidance(cv::Mat& segmentation_mask,
                                             std::vector<cv::Point>& midCurve)
{
    try
    {
        avoidance->buildOccupancy(segmentation_mask);
        avoidance->buildTrajectoryGrid(midCurve);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error in obstacle avoidance lalala: " << e.what()
                  << std::endl;
    }

    try
    {
        if (avoidance->detectAllCollisions())
        {
            std::vector<cv::Point> adjustedTrajectory =
                avoidance->adjustTrajectory(midCurve);
            // Replace original trajectory with adjusted one
            midCurve.clear();
            midCurve = adjustedTrajectory;

            cv::Scalar midCurveColor = cv::Scalar(0, 255, 0); // White
            for (size_t i = 1; i < midCurve.size(); i++)
            {
                cv::line(allPolylinesViz_, midCurve[i - 1], midCurve[i],
                         midCurveColor, 3);
            }

            // Sample point at 30% of height from bottom
            int targetY = frameHeight_ - (0.65 * frameHeight_);

            // Find closest point in adjusted trajectory at target Y
            cv::Point adjustedPoint =
                findClosestPointAtY(adjustedTrajectory, targetY);

            // Find lane boundaries at same Y level
            cv::Point leftLanePoint =
                findClosestPointAtY(prevLeftCurve, targetY);
            cv::Point rightLanePoint =
                findClosestPointAtY(prevRightCurve, targetY);

            if (adjustedPoint.x < leftLanePoint.x)
            {
                // Trajectory crossed left - swap lanes
                prevRightCurve = prevLeftCurve;
                prevLeftCurve.clear();
                std::cout << "Trajectory crossed right" << std::endl;
            }
            else if (adjustedPoint.x > rightLanePoint.x)
            {
                // Trajectory crossed right - swap lanes
                prevLeftCurve = prevRightCurve;
                prevRightCurve.clear();
                std::cout << "Trajectory crossed right" << std::endl;
            }
        }
        avoidance->visualizeGrid(&midCurve, segmentation_mask);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error in obstacle avoidance: " << e.what() << std::endl;
    }
}

cv::Point
TrajectoryDefinition::findClosestPointAtY(const std::vector<cv::Point>& curve,
                                          int targetY)
{
    if (curve.empty())
    {
        return cv::Point(0, targetY);
    }

    cv::Point closestPoint = curve[0];
    int minDistance        = std::abs(curve[0].y - targetY);

    for (const auto& point : curve)
    {
        int distance = std::abs(point.y - targetY);
        if (distance < minDistance)
        {
            minDistance  = distance;
            closestPoint = point;
        }
    }

    return closestPoint;
}

void TrajectoryDefinition::adaptiveSpeedControl(
    cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve)
{
    if (midCurve.empty())
    {
        cv::putText(allPolylinesViz_, "No trajectory for ACC",
                    cv::Point(20, 140), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 255), 2);
        return;
    }

    // Calculate recommended speed
    float recommendedSpeed =
        accontroller->calculateAdaptiveSpeed(segmentation_mask, midCurve);

    // Get obstacle info for visualization
    int obstacleDistance  = accontroller->getCurrentObstacleDistance();
    float obstacleSpeed   = accontroller->getObstacleSpeed();
    bool obstacleDetected = accontroller->isObstacleDetected();
    cv::Point obstaclePos = accontroller->getObstaclePosition();

    // Update global distance variable
    distanceToObstacle_ = obstacleDetected ? obstacleDistance : frameHeight_;

    // Visualize obstacle and information
    if (obstacleDetected)
    {
        cv::circle(allPolylinesViz_, obstaclePos, 15, cv::Scalar(255, 165, 0),
                   -1);
        cv::putText(allPolylinesViz_, "OBS", obstaclePos + cv::Point(-15, 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),
                    2);

        // Display ACC info
        std::string accInfo =
            "ACC: " + std::to_string(obstacleDistance) + "px, " +
            std::to_string(obstacleSpeed).substr(0, 5) + "px/s, " +
            std::to_string(recommendedSpeed).substr(0, 4);
        cv::putText(allPolylinesViz_, accInfo, cv::Point(20, 140),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
    else
    {
        cv::putText(allPolylinesViz_, "ACC: Clear path", cv::Point(20, 140),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    if (recommendedSpeed >= 0)
    {
        publishACC(std::to_string(recommendedSpeed));
    }
    else
    {
    }
}

void TrajectoryDefinition::mpcDebug(void)
{
    // Draw the predicted trajectory as a green polyline
    if (mpcPoints_.size() > 1)
    {
        for (size_t i = 1; i < mpcPoints_.size(); ++i)
        {
            cv::line(allPolylinesViz_, mpcPoints_[i - 1], mpcPoints_[i],
                     cv::Scalar(0, 255, 0), 2);
        }
    }
}

void TrajectoryDefinition::publishEmergencyBrake(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    emergency_brake_publisher_->put(std::move(buf));
}

void TrajectoryDefinition::publishCoeffs(std::vector<cv::Point>& curve)
{
    std::sort(curve.begin(), curve.end(),
              [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });

    std::vector<double> x_values, y_values;

    cv::Mat coeffs;
    for (const auto& pt : curve)
    {
        double y_prime =
            height_ - static_cast<double>(pt.y); // Flip and shift y
        double x_prime = static_cast<double>(pt.x) - width_ / 2; // Shift x
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
            double y_prime         = y_values[i];
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

        const auto len    = coeffs_str.size() + 1;
        auto alloc_result = provider_->alloc_gc_defrag_blocking(
            len, zenoh::AllocAlignment({0}));
        zenoh::ZShmMut&& buf =
            std::get<zenoh::ZShmMut>(std::move(alloc_result));
        memcpy(buf.data(), coeffs_str.c_str(), len);
        coeffs_publisher_->put(std::move(buf));
    }
    else
    {
        std::cerr << "Not enough points to calculate coefficients" << std::endl;
        return;
    }
}

void TrajectoryDefinition::publishLKAS(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    lkas_publisher_->put(std::move(buf));
}

void TrajectoryDefinition::publishSAE2Disable(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    sae_2_disable_publisher_->put(std::move(buf));
}

void TrajectoryDefinition::publishACC(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    acc_speed_publisher_->put(std::move(buf));
}

void TrajectoryDefinition::publishAutonomyEnvEnable(
    const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    autonomy_env_enable_->put(std::move(buf));
}

void TrajectoryDefinition::publishIPMFrame(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    // ipm_frame_publisher_->put(std::move(buf));
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
#include "TrajectoryDefinition.hpp"
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

TrajectoryDefinition::TrajectoryDefinition(
    std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    speed_lock_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/Speed/Lock")));
    try
    {
        this->canBus = new CAN();
        this->canBus->init("/dev/spidev0.0");
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing CAN" << e.what() << std::endl;
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
        float cameraHeight  = 0.137f; // meters
        float cameraPitch   = 20.0f;  // degrees down from horizontal
        float horizontalFOV = 100.0f; // degrees
        float img_height    = static_cast<float>(HEIGHT);
        float img_width     = static_cast<float>(WIDTH);
        float h_fov_rad     = horizontalFOV * CV_PI / 180.0f;
        float verticalFOV =
            2.0f *
            std::atan((img_height / img_width) * std::tan(h_fov_rad / 2.0f)) *
            180.0f / CV_PI;
        float nearDistance = 0.01f; // meters
        float farDistance  = 0.45f; // meters
        float laneWidth    = 1.4f;  // meters
        cv::Size bevSize   = cv::Size(WIDTH, HEIGHT);
        cv::Size origSize  = cv::Size(WIDTH, HEIGHT);

        this->ipm = new IPM();
        this->ipm->init(origSize, bevSize);
        this->ipm->calibrateFromCamera(cameraHeight, cameraPitch, horizontalFOV,
                                       verticalFOV, nearDistance, farDistance,
                                       laneWidth);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing IPM" << e.what() << std::endl;
    }
    

    try
    {
        this->avoidance = new ObstacleAvoidance(WIDTH, HEIGHT, 4);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing ObstacleAvoidance" << e.what()
                  << std::endl;
    }

    // Create an OpenCV CUDA stream
    cv_stream = cv::cuda::Stream();

    publisher_ = std::make_shared<LaneDetectorPublisher>(session_);
}

TrajectoryDefinition::~TrajectoryDefinition()
{
    delete kalmanFilter;
    delete canBus;
    delete ipm;
    delete avoidance;
}

void TrajectoryDefinition::process(cv::Mat& frame, cv::Mat& binary_mask,
                                   cv::Mat& class_mask)
{
    cv::Mat ipm_binary_mask = ipm->applyIPM(binary_mask);
    cv::Mat ipm_class_mask  = ipm->applyIPM(class_mask);
    cv::Mat ipm_frame       = ipm->applyIPM(frame);

    cv::Mat resized_ipm_binary_mask;
    cv::resize(ipm_binary_mask, resized_ipm_binary_mask, frame.size(), 0, 0,
               cv::INTER_NEAREST);

    cv::Mat resized_ipm_class_mask;
    cv::resize(ipm_class_mask, resized_ipm_class_mask, frame.size(), 0, 0,
               cv::INTER_NEAREST);

    cv::Mat resized_ipm_frame;
    cv::resize(ipm_frame, resized_ipm_frame, frame.size(), 0, 0,
               cv::INTER_NEAREST);

    resized_ipm_frame.copyTo(frame);

    createLanes(frame, resized_ipm_binary_mask, resized_ipm_class_mask);
    cv::addWeighted(frame, 0.7, resized_ipm_class_mask, 0.3, 0, frame);
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

    lanePolylines = clusterLaneMask(binary_mask, 30, 40, 6);

    float maxHorizontalDistance = frameWidth_ * 0.15;  // 15% of frame width
    float maxVerticalGap        = frameHeight_ * 0.40; // 35% of frame height
    mergeLaneComponents(lanePolylines, maxHorizontalDistance, maxVerticalGap);

    drawPolyLanes(lanePolylines);

    if (lanePolylines.size() == 2)
    {
        cv::Point lowestPoint1(-1, -1);
        cv::Point lowestPoint2(-1, -1);

        // Find lowest point in first polyline
        for (const auto& pt : lanePolylines[0])
        {
            if (pt.y > lowestPoint1.y)
            {
                lowestPoint1 = pt;
            }
        }

        // Find lowest point in second polyline
        for (const auto& pt : lanePolylines[1])
        {
            if (pt.y > lowestPoint2.y)
            {
                lowestPoint2 = pt;
            }
        }

        // Debug visualization of lowest points
        cv::circle(allPolylinesViz_, lowestPoint1, 8, cv::Scalar(255, 0, 255),
                   -1);
        cv::circle(allPolylinesViz_, lowestPoint2, 8, cv::Scalar(0, 255, 255),
                   -1);

        // Compare x-coordinates to determine left/right
        if (lowestPoint1.x < lowestPoint2.x)
        {
            leftCurve  = lanePolylines[0];
            rightCurve = lanePolylines[1];

            // Debug text
            std::string leftText  = "Left: " + std::to_string(lowestPoint1.x);
            std::string rightText = "Right: " + std::to_string(lowestPoint2.x);
            cv::putText(
                allPolylinesViz_, leftText, lowestPoint1 + cv::Point(10, 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
            cv::putText(
                allPolylinesViz_, rightText, lowestPoint2 + cv::Point(10, 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }
        else
        {
            leftCurve  = lanePolylines[1];
            rightCurve = lanePolylines[0];

            // Debug text
            std::string leftText  = "Left: " + std::to_string(lowestPoint2.x);
            std::string rightText = "Right: " + std::to_string(lowestPoint1.x);
            cv::putText(
                allPolylinesViz_, leftText, lowestPoint2 + cv::Point(10, 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
            cv::putText(
                allPolylinesViz_, rightText, lowestPoint1 + cv::Point(10, 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }

        // Limit the maximum curve drift from center
        if (!leftCurve.empty() && !rightCurve.empty())
        {
            float centerX = frameWidth_ / 2.0f;
            float maxOffsetDistance =
                frameWidth_ *
                0.3f; // Maximum allowed offset (30% of frame width)

            // Calculate current lane midpoint at each y-level
            for (size_t i = 0;
                 i < std::min(leftCurve.size(), rightCurve.size()); i++)
            {
                size_t leftIdx = i * leftCurve.size() /
                                 std::min(leftCurve.size(), rightCurve.size());
                size_t rightIdx = i * rightCurve.size() /
                                  std::min(leftCurve.size(), rightCurve.size());

                float midX =
                    (leftCurve[leftIdx].x + rightCurve[rightIdx].x) / 2.0f;
                float offset = midX - centerX;

                // If offset exceeds limit, adjust both lane curves
                if (std::abs(offset) > maxOffsetDistance)
                {
                    float adjustment =
                        offset -
                        (offset > 0 ? maxOffsetDistance : -maxOffsetDistance);

                    // Apply adjustment to this point in both curves
                    leftCurve[leftIdx].x -= adjustment;
                    rightCurve[rightIdx].x -= adjustment;
                }
            }
        }

        kalmanFilter->updateLeftLaneFilter(leftCurve);
        kalmanFilter->updateRightLaneFilter(rightCurve);

        defineTrajectoryCurve(midCurve, leftCurve, rightCurve);
        kalmanFilter->updateMiddleLaneFilter(midCurve);

        prevLeftCurve  = leftCurve;
        prevRightCurve = rightCurve;

        leftLaneLastUpdatedFrame  = currentFrame;
        rightLaneLastUpdatedFrame = currentFrame;
    }
    else if (lanePolylines.size() == 1)
    {
        bool isLeftLane = checkIfLeftLane(lanePolylines);

        if (isLeftLane)
        {
            leftCurve = lanePolylines[0];

            prevLeftCurve            = leftCurve;
            leftLaneLastUpdatedFrame = currentFrame;

            rightCurve =
                kalmanFilter->predictRightLaneCurve(frameHeight_, frameWidth_);

            checkPredicedCurve(rightCurve, leftCurve, true);

            defineTrajectoryCurve(midCurve, leftCurve, rightCurve);
            kalmanFilter->updateMiddleLaneFilter(midCurve);
        }
        else
        {
            rightCurve = lanePolylines[0];

            prevRightCurve            = rightCurve;
            rightLaneLastUpdatedFrame = currentFrame;

            leftCurve =
                kalmanFilter->predictLeftLaneCurve(frameHeight_, frameWidth_);

            checkPredicedCurve(leftCurve, rightCurve, false);

            defineTrajectoryCurve(midCurve, leftCurve, rightCurve);
            kalmanFilter->updateMiddleLaneFilter(midCurve);
        }

        std::string statusMsg = isLeftLane ? "Using kalmanFilter RIGHT lane"
                                           : "Using kalmanFilter LEFT lane";
        cv::putText(allPolylinesViz_, statusMsg, cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    else
    {
        midCurve =
            kalmanFilter->predictMiddleLaneCurve(frameHeight_, frameWidth_);

        cv::Scalar midCurveColor = cv::Scalar(255, 255, 255); // White
        for (size_t i = 1; i < midCurve.size(); i++)
        {
            cv::line(allPolylinesViz_, midCurve[i - 1], midCurve[i],
                     midCurveColor, 3);
        }
    }

    createMidPointError(midCurve);
    
    obstacleAvoidance(class_mask, midCurve);

    checkForwardCollision(class_mask, midCurve);


    allPolylinesViz_.copyTo(frame);
}

std::vector<std::vector<cv::Point>>
TrajectoryDefinition::clusterLaneMask(const cv::Mat& laneMask, int kernelSize,
                                      int minArea, int maxLanes)
{
    static cv::Mat verticalKernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernelSize, kernelSize * 3));
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

    // Use partial sort instead of full sort when number of valid components
    // bigger than maxLanes
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

    // Reserve capacity for output
    std::vector<std::vector<cv::Point>> lanePolylines;
    lanePolylines.reserve(validComponents.size());

    // Process each lane with optimized extraction
    for (const auto& comp : validComponents)
    {
        int compIdx = comp.first;

        // Extract points more efficiently using row pointers
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
            for (size_t j = i + 1; j < lanePolylines.size() && !mergePerformed;
                 j++)
            {
                // Compute the y-range and x-average of both polylines
                int minY1 = INT_MAX, maxY1 = 0;
                int minY2 = INT_MAX, maxY2 = 0;
                float avgX1 = 0, avgX2 = 0;

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

                // Check if they're horizontally aligned (similar X position)
                float hDist = std::abs(avgX1 - avgX2);

                // Calculate the vertical gap between segments
                float verticalGap;
                if (maxY1 < minY2)
                {
                    // First segment is above second segment
                    verticalGap = minY2 - maxY1;
                }
                else if (maxY2 < minY1)
                {
                    // Second segment is above first segment
                    verticalGap = minY1 - maxY2;
                }
                else
                {
                    // Segments overlap vertically - no gap
                    verticalGap = 0;
                }

                // Also check if they have similar directions (optional but
                // recommended)
                bool similarDirection = true;
                if (!lanePolylines[i].empty() && lanePolylines[i].size() > 1 &&
                    !lanePolylines[j].empty() && lanePolylines[j].size() > 1)
                {
                    // Calculate direction of first segment
                    cv::Point dir1 =
                        lanePolylines[i].back() - lanePolylines[i].front();

                    // Calculate direction of second segment
                    cv::Point dir2 =
                        lanePolylines[j].back() - lanePolylines[j].front();

                    // Calculate dot product to check direction similarity
                    float dotProduct = dir1.x * dir2.x + dir1.y * dir2.y;
                    similarDirection =
                        (dotProduct >
                         0); // Positive dot product means similar direction
                }

                // Merge if horizontally close AND reasonable vertical gap AND
                // similar direction
                if (hDist <= maxHorizontalDist &&
                    verticalGap <= maxVerticalGap && similarDirection)
                {
                    // Merge polylines
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

    // Sort points in each polyline by y-coordinate
    for (auto& polyline : lanePolylines)
    {
        std::sort(polyline.begin(), polyline.end(),
                  [](const cv::Point& a, const cv::Point& b)
                  { return a.y < b.y; });
    }

    if (lanePolylines.size() > 2)
    {
        lanePolylines.resize(2);
    }

    float minLaneWidth = frameWidth_ * 0.1f; // At least 20% of frame width

    if (lanePolylines.size() == 2)
    {
        if (!validateLaneSeparation(lanePolylines, minLaneWidth))
        {
            // Lanes are too close - keep only the stronger one
            if (lanePolylines[0].size() > lanePolylines[1].size())
            {
                lanePolylines.erase(lanePolylines.begin() + 1);
            }
            else
            {
                lanePolylines.erase(lanePolylines.begin());
            }
        }
    }
}

void TrajectoryDefinition::drawPolyLanes(
    std::vector<std::vector<cv::Point>> lanePolylines)
{
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),   // Blue
        cv::Scalar(0, 255, 0),   // Green
        cv::Scalar(0, 0, 255),   // Red
        cv::Scalar(255, 255, 0), // Cyan
        cv::Scalar(255, 0, 255), // Magenta
        cv::Scalar(0, 255, 255)  // Yellow
    };

    // Draw each polyline with a different color
    for (size_t i = 0; i < lanePolylines.size(); i++)
    {
        cv::Scalar color = colors[i % colors.size()];
        for (size_t j = 1; j < lanePolylines[i].size(); j++)
        {
            cv::line(allPolylinesViz_, lanePolylines[i][j - 1],
                     lanePolylines[i][j], color, 2);
        }

        // Add a label for each polyline
        if (!lanePolylines[i].empty())
        {
            std::string label = "Lane " + std::to_string(i + 1);
            cv::putText(allPolylinesViz_, label, lanePolylines[i][0],
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
    }

    // Display the number of polylines found
    std::string countText =
        "Polylines: " + std::to_string(lanePolylines.size());
    cv::putText(allPolylinesViz_, countText, cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

float TrajectoryDefinition::calculateLaneDistance(
    const std::vector<cv::Point>& lane1, const std::vector<cv::Point>& lane2)
{
    // Create normalized Y-position mapping of lane points
    std::map<int, cv::Point> lane1Points;
    std::map<int, cv::Point> lane2Points;

    // Normalize Y values to 0-100 range
    for (const auto& pt : lane1)
    {
        int normY          = (pt.y * 100) / 480; // Assuming 480 is max height
        lane1Points[normY] = pt;
    }

    for (const auto& pt : lane2)
    {
        int normY          = (pt.y * 100) / 480; // Assuming 480 is max height
        lane2Points[normY] = pt;
    }

    // Calculate average distance between lanes at matching Y positions
    float totalDist = 0;
    int matchCount  = 0;

    for (const auto& p1 : lane1Points)
    {
        int y = p1.first;
        if (lane2Points.find(y) != lane2Points.end())
        {
            // Calculate Euclidean distance
            float dist = cv::norm(p1.second - lane2Points[y]);
            totalDist += dist;
            matchCount++;
        }
    }

    return (matchCount > 0) ? (totalDist / matchCount) : FLT_MAX;
}

bool TrajectoryDefinition::validateLaneSeparation(
    const std::vector<std::vector<cv::Point>>& lanePolylines,
    float minLaneWidth)
{
    if (lanePolylines.size() < 2)
        return true;

    float avgDistance =
        calculateLaneDistance(lanePolylines[0], lanePolylines[1]);

    return avgDistance >= minLaneWidth;
}

void TrajectoryDefinition::checkPredicedCurve(
    std::vector<cv::Point>& predictedCurve,
    const std::vector<cv::Point>& realLane, bool isLeftLane)
{
    float avgMiddleX        = 0;
    float avgDetectedX      = 0;
    float expectedHalfWidth = frameWidth_ * 0.50f;
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
        expectedMiddleX = avgDetectedX + expectedHalfWidth;
    }
    else
    {
        expectedMiddleX = avgDetectedX - expectedHalfWidth;
    }

    float error = std::abs(avgMiddleX - expectedMiddleX);

    if (error > frameWidth_ * 0.05f)
    {
        cv::putText(allPolylinesViz_, "Invalid curve prediction - using offset",
                    cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1);

        predictedCurve.clear();
        predictedCurve.reserve(realLane.size());
        if (isLeftLane)
        {
            for (const auto& pt : realLane)
            {
                predictedCurve.push_back(
                    cv::Point(pt.x + expectedHalfWidth, pt.y));
            }
        }
        else
        {
            for (const auto& pt : realLane)
            {
                predictedCurve.push_back(
                    cv::Point(pt.x - expectedHalfWidth, pt.y));
            }
        }

        if (isLeftLane)
        {
            kalmanFilter->updateRightLaneFilter(predictedCurve);
        }
        else
        {
            kalmanFilter->updateLeftLaneFilter(predictedCurve);
        }
    }
}

void TrajectoryDefinition::defineTrajectoryCurve(
    std::vector<cv::Point>& midCurve, std::vector<cv::Point>& leftCurve,
    std::vector<cv::Point>& rightCurve)
{
    // Make sure we have equal length curves by resampling if needed
    int numPoints = std::min(leftCurve.size(), rightCurve.size());
    for (int i = 0; i < numPoints; i++)
    {
        size_t leftIdx  = i * leftCurve.size() / numPoints;
        size_t rightIdx = i * rightCurve.size() / numPoints;

        int midX = (leftCurve[leftIdx].x + rightCurve[rightIdx].x) / 2;
        int midY = (leftCurve[leftIdx].y + rightCurve[rightIdx].y) / 2;
        midCurve.push_back(cv::Point(midX, midY));
    }

    // Draw middle lane curve with white color and thicker line
    cv::Scalar midCurveColor = cv::Scalar(255, 255, 255); // White
    for (size_t i = 1; i < midCurve.size(); i++)
    {
        cv::line(allPolylinesViz_, midCurve[i - 1], midCurve[i], midCurveColor,
                 3);
    }
}

void TrajectoryDefinition::defineTrajectoryPolyline(
    std::vector<cv::Point>& midCurve)
{
    // Sort points by y coordinate (from top to bottom)
    std::sort(midCurve.begin(), midCurve.end(),
              [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });

    // Prepare data for polynomial fitting
    std::vector<double> x_values, y_values;
    for (const auto& pt : midCurve)
    {
        y_values.push_back(static_cast<double>(pt.y));
        x_values.push_back(static_cast<double>(pt.x));
    }

    // Fit a 3rd degree polynomial to the points
    cv::Mat coeffs;
    if (x_values.size() >= 4)
    { // Need at least 4 points for 3rd degree polynomial
        cv::Mat y_mat(y_values), x_mat;

        // Create Vandermonde matrix for polynomial fitting
        x_mat.create(y_values.size(), 4, CV_64F);
        for (int i = 0; i < x_mat.rows; i++)
        {
            x_mat.at<double>(i, 0) = 1.0;
            x_mat.at<double>(i, 1) = y_values[i];
            x_mat.at<double>(i, 2) = y_values[i] * y_values[i];
            x_mat.at<double>(i, 3) = y_values[i] * y_values[i] * y_values[i];
        }

        // Solve for polynomial coefficients using least squares
        cv::solve(x_mat, cv::Mat(x_values), coeffs, cv::DECOMP_SVD);

        // Clear existing midCurve and create smooth curve from polynomial
        midCurve.clear();

        // Sample more points along the polynomial for a smoother curve
        int numSamples =
            frameHeight_ / 100; // Sample every 5 pixels in y-direction
        for (int y = 0; y < frameHeight_; y += numSamples)
        {
            // if (y > frameHeight_ * 0.5) {  // Only use lower half of screen
            // for trajectory Evaluate polynomial: x = a + by + cy² + dy³
            double yVal = static_cast<double>(y);
            double xVal = coeffs.at<double>(0) + coeffs.at<double>(1) * yVal +
                          coeffs.at<double>(2) * yVal * yVal +
                          coeffs.at<double>(3) * yVal * yVal * yVal;

            // Constrain x within frame boundaries
            xVal =
                std::max(0.0, std::min(static_cast<double>(frameWidth_), xVal));

            midCurve.push_back(cv::Point(static_cast<int>(xVal), y));
            // }
        }
    }

    // Draw middle lane curve with white color and thicker line
    cv::Scalar midCurveColor = cv::Scalar(255, 255, 255); // White
    for (size_t i = 1; i < midCurve.size(); i++)
    {
        cv::line(allPolylinesViz_, midCurve[i - 1], midCurve[i], midCurveColor,
                 3);
    }
}

void TrajectoryDefinition::createMidPointError(std::vector<cv::Point>& midCurve)
{
    cv::Point midPoint;
    int height = frameHeight_;
    int width  = frameWidth_;

    if (!midCurve.empty())
    {
        int targetY = height - (1.5 * height / 3); // 1/3 up from bottom

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

        float centerX  = width / 2;
        float rawError = (midPoint.x - centerX) / (width / 2.0f);

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
        cv::putText(allPolylinesViz_, statusMsg, cv::Point(60, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
}

bool TrajectoryDefinition::checkIfLeftLane(
    const std::vector<std::vector<cv::Point>>& lanePolylines)
{
    cv::Point lowestPoint(-1, -1);
    int centerX = frameWidth_ / 2;
    float avgX  = 0;

    // Find lowest point and average X position
    for (const auto& pt : lanePolylines[0])
    {
        if (pt.y > lowestPoint.y)
        {
            lowestPoint = pt;
        }
        avgX += pt.x;
    }
    avgX /= lanePolylines[0].size();

    // Draw detected lane's lowest point
    cv::circle(allPolylinesViz_, lowestPoint, 8, cv::Scalar(255, 0, 255), -1);

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
                ? calculateLaneDistance(lanePolylines[0], prevLeftCurve)
                : FLT_MAX;

        float rightDistance =
            hasValidRightMemory
                ? calculateLaneDistance(lanePolylines[0], prevRightCurve)
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

    defineTrajectoryPolyline(midCurve);

    const int zoneWidth = WIDTH * 0.20; // Width of zone around trajectory
    int total_pixels    = 0;
    int road_pixels     = 0;

    std::vector<cv::Point> rightPoints;
    std::vector<cv::Point> polygonPoints;

    // Calculate left and right boundaries of the trajectory zone
    for (size_t i = 0; i < midCurve.size() - 2; i += 2)
    { // Skip points for efficiency
        // Only check lower half of the curve (close to the vehicle)
        if (midCurve[i].y < HEIGHT * 0.7)
            continue;

        // Calculate direction vector between points
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

        // Normalize direction
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
    for (int y = 0; y < polygonMask.rows; y++)
    {
        for (int x = 0; x < polygonMask.cols; x++)
        {
            // Only check pixels inside the polygon
            if (polygonMask.at<uchar>(y, x) > 0)
            {
                total_pixels++;
                cv::Vec3b pixel = segmentation_mask.at<cv::Vec3b>(y, x);

                if (pixel == cv::Vec3b(128, 64, 128) ||
                    pixel == cv::Vec3b(0, 0, 0))
                {
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

    // Display road percentage
    std::string roadText =
        "Road: " + std::to_string(int(road_percentage * 100)) + "%" +
        " Total: " + std::to_string(total_pixels);
    cv::putText(allPolylinesViz_, roadText, cv::Point(20, 120),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                danger_detected ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0),
                2);

    if (danger_detected)
    {
        cv::putText(allPolylinesViz_, "OBSTACLE DETECTED!",
                    cv::Point(frameWidth_ / 2 - 150, frameHeight_ / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);

        std::cout << "\033[1;31m*** WARNING: OBSTACLE DETECTED! STOPPING "
                     "VEHICLE ***\033[0m"
                  << std::endl;

        is_emergency_stop = true;

        publishSpeedLock("1");
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

void TrajectoryDefinition::obstacleAvoidance(const cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve)
{
    (void) midCurve;
    avoidance->buildOccupancy(segmentation_mask);
    // // Get a color frame to use for visualization
    cv::Mat visualization;
    segmentation_mask.copyTo(visualization);
    
    std::cout << "Occupancy grid built successfully." << std::endl;
    // Visualize the grid
    avoidance->visualizeGrid(visualization);
}
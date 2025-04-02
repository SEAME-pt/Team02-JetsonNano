#include "LaneDetectorCV.hpp"
#include <sys/time.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace zenoh;

void LaneDetectorCV::setCalibrationParameters(void)
{
    // calibration parameters file path here this should be saved on the class
    // to be used in RUN method (function)
    FileStorage fs("calibration.yml", FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Failed to open calibration.yml" << endl;
        return;
    }
    Mat tempMatrix, tempCoeffs;
    fs["CameraMatrix"] >> tempMatrix;
    fs["DistCoeffs"] >> tempCoeffs;
    fs.release();
    this->cameraMatrix = tempMatrix;
    this->distCoeffs   = tempCoeffs;
}

LaneDetectorCV::LaneDetectorCV(const std::string& pipeline,
                               std::shared_ptr<zenoh::Session> session)
    : cap(pipeline, cv::CAP_GSTREAMER), session_(session),
      prevLeftLine(0, 0, 0, 0), prevRightLine(0, 0, 0, 0),
      prevMidLine(0, 0, 0, 0), laneWidthEstimate(0.0), firstFrame(true),
      frame_count(0), FRAME_SKIP(2)

{
    if (!cap.isOpened())
    {
        throw std::runtime_error("Error opening video stream");
    }

    // set kalman filter status to false
    kfInitialized = false;

    // Set camera buffer size
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Initialize the lane detector publisher
    publisher_ = std::make_shared<LaneDetectorPublisher>(session_);
    const std::string canDevice = "/dev/spidev0.0";
    this->canBus     = new CAN();
    this->canBus->init(canDevice);
}

LaneDetectorCV::~LaneDetectorCV()
{
    cap.release();
    destroyAllWindows();
}

Mat LaneDetectorCV::regionOfInterest(const Mat& img,
                                     const vector<Point>& vertices)
{
    Mat mask = Mat::zeros(img.size(), img.type());
    fillPoly(mask, vector<vector<Point>>{vertices}, Scalar(255));
    Mat masked;
    bitwise_and(img, mask, masked);
    return masked;
}

// Polynomial fitting using OpenCV
Mat LaneDetectorCV::polyfit(const Mat& y_vals, const Mat& x_vals, int degree)
{
    // Check if we have enough points for the requested degree
    if (y_vals.rows < degree + 1)
    {
        // Not enough points - reduce degree or return empty matrix
        if (y_vals.rows < 2)
        {
            return Mat();
        }
        // Adjust degree based on available points
        degree = y_vals.rows - 1;
    }

    // Ensure data is in the right format (CV_64F)
    Mat y_vals_64f, x_vals_64f;
    y_vals.convertTo(y_vals_64f, CV_64F);
    x_vals.convertTo(x_vals_64f, CV_64F);

    // Create the design matrix with appropriate dimensions
    Mat A = Mat::zeros(y_vals_64f.rows, degree + 1, CV_64F);

    // Fill the design matrix
    for (int i = 0; i < y_vals_64f.rows; i++)
    {
        for (int j = 0; j <= degree; j++)
        {
            A.at<double>(i, j) = pow(y_vals_64f.at<double>(i), degree - j);
        }
    }

    // Check for invalid values (NaN, Inf)
    for (int i = 0; i < A.rows; i++)
    {
        for (int j = 0; j < A.cols; j++)
        {
            if (cvIsNaN(A.at<double>(i, j)) || cvIsInf(A.at<double>(i, j)))
            {
                A.at<double>(i, j) = 0;
            }
        }
    }

    // Solve the system using SVD for better stability
    Mat coeffs;
    try
    {
        solve(A, x_vals_64f, coeffs, DECOMP_SVD);
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "Error in polyfit: " << e.what() << std::endl;
        return Mat();
    }

    return coeffs;
}

double LaneDetectorCV::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

// Extrapolate a polynomial curve from lane line segments
vector<Point> LaneDetectorCV::extrapolatePolynomialCurve(const vector<Vec4i>& laneLines)
{
    // Extract all points from line segments
    vector<Point2f> points;
    for (auto line : laneLines)
    {
        points.push_back(Point2f(line[0], line[1]));
        points.push_back(Point2f(line[2], line[3]));
    }

    if (points.empty() ||
        points.size() < 3) // Need at least 3 points for quadratic
        return vector<Point>();

    // Convert to vectors for polynomial fitting
    vector<float> x_vals, y_vals;
    for (auto& pt : points)
    {
        x_vals.push_back(pt.x);
        y_vals.push_back(pt.y);
    }

    // Convert to Mat for polyfit
    Mat x_mat(x_vals), y_mat(y_vals);
    int degree = std::min(2, (int)points.size() -
                                 1); // Adjust degree based on points available
    Mat coeffs = polyfit(y_mat, x_mat, degree);

    // Check if coeffs is valid
    if (coeffs.empty() || coeffs.rows < degree + 1)
        return vector<Point>();

    // Generate points along the curve
    vector<Point> curvePoints;
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    for (int y = height; y >= height / 3; y -= 5)
    {
        // Evaluate polynomial based on degree
        double x = 0;
        if (degree == 2 && coeffs.rows >= 3)
        {
            // Quadratic: x = a*y^2 + b*y + c
            x = coeffs.at<double>(0) * y * y + coeffs.at<double>(1) * y +
                coeffs.at<double>(2);
        }
        else if (degree == 1 && coeffs.rows >= 2)
        {
            // Linear: x = a*y + b
            x = coeffs.at<double>(0) * y + coeffs.at<double>(1);
        }
        else
        {
            continue;
        }

        // Check for valid x value before adding point
        if (!std::isnan(x) && !std::isinf(x))
        {
            curvePoints.push_back(Point(round(x), y));
        }
    }

    return curvePoints;
}

Vec4i LaneDetectorCV::extrapolateLine(const vector<Vec4i>& laneLines)
{
    double slopeSum = 0, interceptSum = 0;
    int count  = 0;
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);

    for (auto line : laneLines)
    {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double slope     = (double)(y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;
        slopeSum += slope;
        interceptSum += intercept;
        count++;
    }

    if (count == 0)
        return Vec4i(0, 0, 0, 0);

    double avgSlope     = slopeSum / count;
    double avgIntercept = interceptSum / count;

    // Define endpoints: bottom of ROI (y = height) and top of ROI (y =
    // height/3)
    int yBottom = height;
    int yTop    = height / 3;
    int xBottom = (int)((yBottom - avgIntercept) / avgSlope);
    int xTop    = (int)((yTop - avgIntercept) / avgSlope);

    return Vec4i(xBottom, yBottom, xTop, yTop);
}

void LaneDetectorCV::detect(Mat& frame)
{
    int height = frame.rows;
    int width  = frame.cols;

    // Initialize lane width estimate on first frame.
    if (firstFrame)
    {
        laneWidthEstimate = width * 0.45;
    }

    // 1. Preprocessing: convert to grayscale, blur and detect edges.
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    Mat blur;
    GaussianBlur(gray, blur, Size(5, 5), 0);
    Mat edges;
    Canny(blur, edges, 50, 150);

    // 2. Define ROI covering the lower 2/3 of the frame.
    vector<Point> roiVertices = {Point(0, height), Point(width, height),
                                 Point(width, height / 3),
                                 Point(0, height / 3)};
    Mat maskedEdges           = regionOfInterest(edges, roiVertices);

    // 3. Use the Hough transform to detect line segments.
    vector<Vec4i> lines;

    HoughLinesP(maskedEdges, lines, 1, CV_PI / 180, 20, 20, 30);

    // 4. Separate lines into left and right lanes based on slope and position.
    vector<Vec4i> leftLines, rightLines;
    int midX = width / 2; // Middle of the image

    for (auto line : lines)
    {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

        // Avoid division by zero and filter nearly horizontal lines.
        if (abs(x2 - x1) < 10)
            continue;

        double slope = (double)(y2 - y1) / (x2 - x1);
        if (abs(slope) < 0.5)
            continue; // Filter out nearly horizontal lines

        // Calculate line midpoint
        int lineMidX = (x1 + x2) / 2;

        // Use both slope and position for classification
        // Left lane: negative slope OR on left half of image with steep slope
        // Right lane: positive slope OR on right half of image with steep slope
        if ((slope < 0 && lineMidX < midX * 1.2) || (lineMidX < midX * 0.6))
        {
            leftLines.push_back(line);
        }
        // Right lane: positive slope OR on right half with steep slope
        else if ((slope > 0 && lineMidX > midX * 0.8) ||
                 (lineMidX > midX * 1.4))
        {
            rightLines.push_back(line);
        }
    }

    // Post-classification cleanup - remove outliers using RANSAC or distance
    // thresholding
    if (leftLines.size() > 3)
    {
        // Calculate average position and slope of left lines
        int avgLeftX        = 0;
        double avgLeftSlope = 0;
        for (auto line : leftLines)
        {
            avgLeftX += (line[0] + line[2]) / 2;
            avgLeftSlope += (double)(line[3] - line[1]) / (line[2] - line[0]);
        }
        avgLeftX /= leftLines.size();
        avgLeftSlope /= leftLines.size();

        // Remove outliers based on both position and slope
        vector<Vec4i> filteredLeftLines;
        for (auto line : leftLines)
        {
            int lineMidX = (line[0] + line[2]) / 2;
            double lineSlope =
                (double)(line[3] - line[1]) / (line[2] - line[0]);

            // Only keep lines that are close to both the average position and
            // slope
            if (lineMidX < avgLeftX * 1.3 &&
                abs(lineSlope - avgLeftSlope) < abs(avgLeftSlope * 0.5))
            {
                filteredLeftLines.push_back(line);
            }
        }
        leftLines = filteredLeftLines;
    }

    // Similar logic for right lines
    if (rightLines.size() > 3)
    {
        // Calculate average position and slope of right lines
        int avgRightX        = 0;
        double avgRightSlope = 0;
        for (auto line : rightLines)
        {
            avgRightX += (line[0] + line[2]) / 2;
            avgRightSlope += (double)(line[3] - line[1]) / (line[2] - line[0]);
        }
        avgRightX /= rightLines.size();
        avgRightSlope /= rightLines.size();

        // Remove outliers based on both position and slope
        vector<Vec4i> filteredRightLines;
        for (auto line : rightLines)
        {
            int lineMidX = (line[0] + line[2]) / 2;
            double lineSlope =
                (double)(line[3] - line[1]) / (line[2] - line[0]);

            // Only keep lines that are close to both the average position and
            // slope
            if (lineMidX > avgRightX * 0.7 &&
                abs(lineSlope - avgRightSlope) < abs(avgRightSlope * 0.5))
            {
                filteredRightLines.push_back(line);
            }
        }
        rightLines = filteredRightLines;
    }

    // 5. Extract polynomial curves for left and right lanes
    vector<Point> leftCurve  = extrapolatePolynomialCurve(leftLines);
    vector<Point> rightCurve = extrapolatePolynomialCurve(rightLines);

    // Used to store found points to then represent
    vector<Point2f> leftPoints, rightPoints;
    for (auto line : leftLines)
    {
        leftPoints.push_back(Point2f(line[0], line[1]));
        leftPoints.push_back(Point2f(line[2], line[3]));
    }

    for (auto line : rightLines)
    {
        rightPoints.push_back(Point2f(line[0], line[1]));
        rightPoints.push_back(Point2f(line[2], line[3]));
    }

    // 6. Fallback: if one lane is missing, use previous data or estimate it
    // using lane width Initialize Kalman filter if it's the first good
    // detection
    if (!kfInitialized && !leftCurve.empty() && !rightCurve.empty())
    {
        initKalmanFilters(leftCurve, rightCurve);
    }

    // Use Kalman filter predictions when lanes disappear
    if (kfInitialized)
    {
        // Always predict next state
        cv::Mat leftPredicted  = leftLaneKF.predict();
        cv::Mat rightPredicted = rightLaneKF.predict();

        // If we have actual measurements, use them to update the filter
        if (!leftCurve.empty() && leftCurve.size() >= 3)
        {
            int bottom_idx = 0, mid_idx = leftCurve.size() / 2,
                top_idx = leftCurve.size() - 1;
            cv::Mat measurement =
                (cv::Mat_<float>(3, 1) << leftCurve[bottom_idx].x,
                 leftCurve[mid_idx].x, leftCurve[top_idx].x);
            leftLaneKF.correct(measurement);
        }
        else if (leftCurve.empty())
        {
            // No measurement, use the prediction to generate a curve
            vector<Point> predictedLeftCurve;
            int height = frame.rows;
            if (!rightCurve.empty() && rightCurve.size() >= 3)
            {
                // Use right lane with lane width offset
                predictedLeftCurve.clear();

                // Calculate curvature of the right lane (basic approach: get a
                // curve coefficient)
                int bottom_idx = 0, mid_idx = rightCurve.size() / 2,
                    top_idx = rightCurve.size() - 1;
                // double rightCurvature = 0;

                // Fit polynomial to right lane to get curvature
                vector<Point2f> rightPoints = {
                    Point2f(rightCurve[bottom_idx].x, rightCurve[bottom_idx].y),
                    Point2f(rightCurve[mid_idx].x, rightCurve[mid_idx].y),
                    Point2f(rightCurve[top_idx].x, rightCurve[top_idx].y)};

                vector<float> x_vals, y_vals;
                for (auto& pt : rightPoints)
                {
                    x_vals.push_back(pt.x);
                    y_vals.push_back(pt.y);
                }

                Mat x_mat(x_vals), y_mat(y_vals);
                Mat rightCoeffs = polyfit(y_mat, x_mat, 2);

                // Now use these coefficients but shifted left by lane width
                if (!rightCoeffs.empty() && rightCoeffs.rows >= 3)
                {
                    // Generate left curve by shifting right curve left by lane
                    // width The quadratic coefficient (curvature) stays the
                    // same Only the constant term (x-offset) changes
                    double a =
                        rightCoeffs.at<double>(0); // quadratic term (curvature)
                    double b = rightCoeffs.at<double>(1); // linear term (slope)
                    double c = rightCoeffs.at<double>(2) -
                               laneWidthEstimate; // constant (x-offset)

                    for (int y = height; y >= height / 3; y -= 5)
                    {
                        double x = a * y * y + b * y + c;
                        predictedLeftCurve.push_back(Point(round(x), y));
                    }
                }

                // Use this curve-informed prediction instead of Kalman
                leftCurve = predictedLeftCurve;
            }
            else
            {
                float bottom_x = leftPredicted.at<float>(0);
                float mid_x    = leftPredicted.at<float>(1);
                float top_x    = leftPredicted.at<float>(2);

                // Generate curve points using quadratic interpolation through
                // the three points
                int bottom_y = height;
                int mid_y    = height / 2;
                int top_y    = height / 3;

                // Calculate quadratic coefficients (y = ax^2 + bx + c)
                // Here we're actually fitting x = f(y) since lanes are more
                // vertical than horizontal
                Mat Y      = (Mat_<double>(3, 1) << bottom_y, mid_y, top_y);
                Mat X      = (Mat_<double>(3, 1) << bottom_x, mid_x, top_x);
                Mat coeffs = polyfit(Y, X, 2);

                if (!coeffs.empty() && coeffs.rows >= 3)
                {
                    for (int y = height; y >= height / 3; y -= 5)
                    {
                        double x = coeffs.at<double>(0) * y * y +
                                   coeffs.at<double>(1) * y +
                                   coeffs.at<double>(2);
                        predictedLeftCurve.push_back(Point(round(x), y));
                    }
                    leftCurve = predictedLeftCurve;
                }
            }
        }

        // Similar logic for right curve
        if (!rightCurve.empty() && rightCurve.size() >= 3)
        {
            int bottom_idx = 0, mid_idx = rightCurve.size() / 2,
                top_idx = rightCurve.size() - 1;
            cv::Mat measurement =
                (cv::Mat_<float>(3, 1) << rightCurve[bottom_idx].x,
                 rightCurve[mid_idx].x, rightCurve[top_idx].x);
            rightLaneKF.correct(measurement);
        }
        else if (rightCurve.empty())
        {
            // No measurement, use the prediction to generate a curve
            vector<Point> predictedRightCurve;
            int height = frame.rows;

            if (!leftCurve.empty() && leftCurve.size() >= 3)
            {
                // Use left lane with lane width offset
                predictedRightCurve.clear();

                // Calculate curvature of the left lane (basic approach: get a
                // curve coefficient)
                int bottom_idx = 0, mid_idx = leftCurve.size() / 2,
                    top_idx = leftCurve.size() - 1;
                // double leftCurvature = 0;

                // Fit polynomial to left lane to get curvature
                vector<Point2f> leftPoints = {
                    Point2f(leftCurve[bottom_idx].x, leftCurve[bottom_idx].y),
                    Point2f(leftCurve[mid_idx].x, leftCurve[mid_idx].y),
                    Point2f(leftCurve[top_idx].x, leftCurve[top_idx].y)};

                vector<float> x_vals, y_vals;
                for (auto& pt : leftPoints)
                {
                    x_vals.push_back(pt.x);
                    y_vals.push_back(pt.y);
                }

                Mat x_mat(x_vals), y_mat(y_vals);
                Mat leftCoeffs = polyfit(y_mat, x_mat, 2);

                // Now use these coefficients but shifted right by lane width
                if (!leftCoeffs.empty() && leftCoeffs.rows >= 3)
                {
                    // Generate right curve by shifting left curve right by lane
                    // width The quadratic coefficient (curvature) stays the
                    // same Only the constant term (x-offset) changes
                    double a =
                        leftCoeffs.at<double>(0); // quadratic term (curvature)
                    double b = leftCoeffs.at<double>(1); // linear term (slope)
                    double c = leftCoeffs.at<double>(2) +
                               laneWidthEstimate; // constant (x-offset)

                    for (int y = height; y >= height / 3; y -= 5)
                    {
                        double x = a * y * y + b * y + c;
                        predictedRightCurve.push_back(Point(round(x), y));
                    }
                }

                // Use this curve-informed prediction instead of Kalman
                rightCurve = predictedRightCurve;
            }
            else
            {
                float bottom_x = rightPredicted.at<float>(0);
                float mid_x    = rightPredicted.at<float>(1);
                float top_x    = rightPredicted.at<float>(2);

                // Generate curve points using quadratic interpolation through
                // the three points
                int bottom_y = height;
                int mid_y    = height / 2;
                int top_y    = height / 3;

                // Calculate quadratic coefficients (y = ax^2 + bx + c)
                // Here we're actually fitting x = f(y) since lanes are more
                // vertical than horizontal
                Mat Y      = (Mat_<double>(3, 1) << bottom_y, mid_y, top_y);
                Mat X      = (Mat_<double>(3, 1) << bottom_x, mid_x, top_x);
                Mat coeffs = polyfit(Y, X, 2);

                if (!coeffs.empty() && coeffs.rows >= 3)
                {
                    for (int y = height; y >= height / 3; y -= 5)
                    {
                        double x = coeffs.at<double>(0) * y * y +
                                   coeffs.at<double>(1) * y +
                                   coeffs.at<double>(2);
                        predictedRightCurve.push_back(Point(round(x), y));
                    }
                    rightCurve = predictedRightCurve;
                }
            }
        }
    }

    // Update lane width estimate if both curves have points
    if (!leftCurve.empty() && !rightCurve.empty() && leftCurve.size() > 0 &&
        rightCurve.size() > 0)
    {
        // Find corresponding points at the bottom of the image
        int bottomY = height - 1;
        int leftX = -1, rightX = -1;

        for (const auto& pt : leftCurve)
        {
            if (pt.y == bottomY || (leftX == -1 && pt.y > height * 0.7))
            {
                leftX = pt.x;
                break;
            }
        }

        for (const auto& pt : rightCurve)
        {
            if (pt.y == bottomY || (rightX == -1 && pt.y > height * 0.7))
            {
                rightX = pt.x;
                break;
            }
        }

        if (leftX != -1 && rightX != -1)
        {
            double currentWidth = rightX - leftX;
            laneWidthEstimate   = 0.2 * currentWidth + 0.8 * laneWidthEstimate;
        }
    }

    // 7. Smooth curves by averaging with previous frames
    double alpha = 0.3;
    if (!firstFrame)
    {
        // Apply moving average to left curve if it exists
        if (!leftCurve.empty())
        {
            // Add current curve to history
            leftLaneHistory.push_back(leftCurve);
            if (leftLaneHistory.size() > historySize)
            {
                leftLaneHistory.pop_front();
            }

            // Only apply averaging if we have enough history
            if (leftLaneHistory.size() >= 2)
            {
                // Create a copy of the current curve for averaging
                std::vector<Point> averagedLeftCurve = leftCurve;

                // For each point in the curve
                for (size_t i = 0; i < leftCurve.size(); i++)
                {
                    int sumX = 0, sumY = 0;
                    double totalWeight = 0;

                    // Average across history (weighted, with recent frames
                    // having more weight)
                    for (size_t h = 0; h < leftLaneHistory.size(); h++)
                    {
                        if (i < leftLaneHistory[h].size())
                        {
                            // More recent frames get higher weight
                            double weight = (h + 1.0) / leftLaneHistory.size();
                            sumX += leftLaneHistory[h][i].x * weight;
                            sumY += leftLaneHistory[h][i].y * weight;
                            totalWeight += weight;
                        }
                    }

                    if (totalWeight > 0)
                    {
                        averagedLeftCurve[i].x =
                            static_cast<int>(sumX / totalWeight);
                        averagedLeftCurve[i].y =
                            static_cast<int>(sumY / totalWeight);
                    }
                }
                leftCurve = averagedLeftCurve;
            }

            // Additionally apply dynamic alpha smoothing for even more
            // stability
            if (!prevLeftCurve.empty() &&
                prevLeftCurve.size() == leftCurve.size())
            {
                double curvature = 0;
                if (leftCurve.size() >= 3)
                {
                    // Calculate curvature using all points instead of just
                    // three
                    std::vector<float> x_vals, y_vals;
                    for (const auto& pt : leftCurve)
                    {
                        x_vals.push_back(pt.x);
                        y_vals.push_back(pt.y);
                    }

                    Mat x_mat(x_vals), y_mat(y_vals);
                    Mat coeffs = polyfit(y_mat, x_mat, 2);
                    if (!coeffs.empty() && coeffs.rows >= 3)
                    {
                        curvature = abs(coeffs.at<double>(0));
                    }
                }

                // Lower multiplier for smoother transitions
                double dynamicAlpha = min(0.5, alpha + curvature * 1000);

                for (size_t i = 0; i < leftCurve.size(); i++)
                {
                    leftCurve[i].x = static_cast<int>(
                        dynamicAlpha * leftCurve[i].x +
                        (1 - dynamicAlpha) * prevLeftCurve[i].x);
                    leftCurve[i].y = static_cast<int>(
                        dynamicAlpha * leftCurve[i].y +
                        (1 - dynamicAlpha) * prevLeftCurve[i].y);
                }
            }
        }

        // Apply moving average to right curve if it exists
        if (!rightCurve.empty())
        {
            // Add current curve to history
            rightLaneHistory.push_back(rightCurve);
            if (rightLaneHistory.size() > historySize)
            {
                rightLaneHistory.pop_front();
            }

            // Only apply averaging if we have enough history
            if (rightLaneHistory.size() >= 2)
            {
                // Create a copy of the current curve for averaging
                std::vector<Point> averagedRightCurve = rightCurve;

                // For each point in the curve
                for (size_t i = 0; i < rightCurve.size(); i++)
                {
                    int sumX = 0, sumY = 0;
                    double totalWeight = 0;

                    // Average across history (weighted, with recent frames
                    // having more weight)
                    for (size_t h = 0; h < rightLaneHistory.size(); h++)
                    {
                        if (i < rightLaneHistory[h].size())
                        {
                            // More recent frames get higher weight
                            double weight = (h + 1.0) / rightLaneHistory.size();
                            sumX += rightLaneHistory[h][i].x * weight;
                            sumY += rightLaneHistory[h][i].y * weight;
                            totalWeight += weight;
                        }
                    }

                    if (totalWeight > 0)
                    {
                        averagedRightCurve[i].x =
                            static_cast<int>(sumX / totalWeight);
                        averagedRightCurve[i].y =
                            static_cast<int>(sumY / totalWeight);
                    }
                }
                rightCurve = averagedRightCurve;
            }

            // Additionally apply dynamic alpha smoothing for even more
            // stability
            if (!prevRightCurve.empty() &&
                prevRightCurve.size() == rightCurve.size())
            {
                double curvature = 0;
                if (rightCurve.size() >= 3)
                {
                    // Calculate curvature using all points instead of just
                    // three
                    std::vector<float> x_vals, y_vals;
                    for (const auto& pt : rightCurve)
                    {
                        x_vals.push_back(pt.x);
                        y_vals.push_back(pt.y);
                    }

                    Mat x_mat(x_vals), y_mat(y_vals);
                    Mat coeffs = polyfit(y_mat, x_mat, 2);
                    if (!coeffs.empty() && coeffs.rows >= 3)
                    {
                        curvature = abs(coeffs.at<double>(0));
                    }
                }

                // Lower multiplier for smoother transitions
                double dynamicAlpha = min(0.5, alpha + curvature * 500);

                for (size_t i = 0; i < rightCurve.size(); i++)
                {
                    rightCurve[i].x = static_cast<int>(
                        dynamicAlpha * rightCurve[i].x +
                        (1 - dynamicAlpha) * prevRightCurve[i].x);
                    rightCurve[i].y = static_cast<int>(
                        dynamicAlpha * rightCurve[i].y +
                        (1 - dynamicAlpha) * prevRightCurve[i].y);
                }
            }
        }
    }

    // Save current curves for next frame
    prevLeftCurve  = leftCurve;
    prevRightCurve = rightCurve;

    // 8. Compute mid curve points as average of left and right curves
    vector<Point> midCurve;
    if (!leftCurve.empty() && !rightCurve.empty() &&
        leftCurve.size() == rightCurve.size())
    {
        for (size_t i = 0; i < leftCurve.size(); i++)
        {
            int midX = (leftCurve[i].x + rightCurve[i].x) / 2;
            int midY = (leftCurve[i].y + rightCurve[i].y) / 2;
            midCurve.push_back(Point(midX, midY));
        }
    }

    // Add a member variable to the class to store the previous midPoint
    Point prevMidPoint = Point(-1, -1); // Initialize with an invalid point

    // 9. Compute the midline reference point
    Point midPoint;
    if (!midCurve.empty())
    {
        int targetY = height - (height / 3); // 2/3 up from bottom

        // Find the closest point to our target Y value
        size_t closest_idx = 0;
        int min_distance   = abs(midCurve[0].y - targetY);

        for (size_t i = 1; i < midCurve.size(); i++)
        {
            int distance = abs(midCurve[i].y - targetY);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_idx  = i;
            }
        }

        // Use the point at the found index
        midPoint = midCurve[closest_idx];

        // Apply smoothing to stabilize the midPoint
        if (prevMidPoint.x != -1 && prevMidPoint.y != -1)
        {                     // Check if prevMidPoint is valid
            float gama = 0.8; // Smoothing factor (adjust as needed)
            midPoint.x = static_cast<int>(gama * midPoint.x +
                                          (1 - gama) * prevMidPoint.x);
            midPoint.y = static_cast<int>(gama * midPoint.y +
                                          (1 - gama) * prevMidPoint.y);
        }

        // Draw a horizontal line at the target Y for visualization
        line(frame, Point(0, targetY), Point(width, targetY),
             Scalar(0, 255, 255), 1);
    }
    else
    {
        // Fallback to center of image
        midPoint = Point(width / 2, height * 2 / 3);
    }

    // Update prevMidPoint for the next frame
    prevMidPoint = midPoint;

    prevMidCurve = midCurve;
    firstFrame   = false;

    // Calculate lateral error
    float centerX      = width / 2;
    float lateralError = midPoint.x - centerX;
    float divider      = width / 2;
    lateralError       = lateralError / divider;

    // Publish camera error
    if (publisher_)
    {
        publisher_->publishCameraError(lateralError);
        //publisher_->publishLanes(leftCurve, rightCurve);
        if (!leftCurve.empty() && !rightCurve.empty())
            sendCoefs(leftCurve, rightCurve);
    }

    // 10. Draw the detected lane curves and midpoint
    Mat lineImage = Mat::zeros(frame.size(), frame.type());

    // Draw left curve
    if (!leftCurve.empty())
    {
        for (size_t i = 1; i < leftCurve.size(); i++)
        {
            line(lineImage, leftCurve[i - 1], leftCurve[i], Scalar(255, 0, 0),
                 5);
        }
    }

    // Draw right curve
    if (!rightCurve.empty())
    {
        for (size_t i = 1; i < rightCurve.size(); i++)
        {
            line(lineImage, rightCurve[i - 1], rightCurve[i], Scalar(0, 255, 0),
                 5);
        }
    }

    // // Draw mid curve (optional)
    // if (!midCurve.empty()) {
    //     for (size_t i = 1; i < midCurve.size(); i++) {
    //         line(lineImage, midCurve[i-1], midCurve[i], Scalar(0, 0, 255),
    //         3);
    //     }
    // }

    // Draw reference point
    circle(lineImage, midPoint, 8, Scalar(0, 0, 255), -1);

    // Draw all detected points to see what's being fitted
    for (const auto& pt : leftPoints)
    {
        circle(lineImage, pt, 4, Scalar(255, 255, 0),
               -1); // Yellow for left points
    }

    for (const auto& pt : rightPoints)
    {
        circle(lineImage, pt, 4, Scalar(0, 255, 255),
               -1); // Cyan for right points
    }

    // 11. Overlay the lane curves and reference point on the original frame.
    addWeighted(frame, 0.8, lineImage, 1.0, 0, frame);
}

void LaneDetectorCV::run()
{
    Mat frame;
    bool mapsInitialized =
        false; // Flag to ensure we compute the maps only once

    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;

        // Compute the undistortion maps once using the frame size from the
        // first frame.
        if (!mapsInitialized && !cameraMatrix.empty() && !distCoeffs.empty())
        {
            Size imageSize = frame.size();
            // Initialize the maps with a rectification transform of identity
            // (no rotation)
            initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                    cameraMatrix, imageSize, CV_16SC2, map1,
                                    map2);
            mapsInitialized = true;
        }
        if (frame_count % FRAME_SKIP == 0)
        {
            Mat undistorted;
            remap(frame, undistorted, map1, map2, INTER_LINEAR);
            frame = undistorted;
            detect(frame);
            // imshow("Lane Detection", frame);
            publisher_->publishCameraFrame(frame);
        }

        frame_count++;
        if (waitKey(1) == 27)
            break;
    }
}

void LaneDetectorCV::initKalmanFilters(const vector<Point>& leftCurve,
                                       const vector<Point>& rightCurve)
{
    // For simplicity, we'll track the bottom, middle, and top points of each
    // lane State: [x_bottom, x_middle, x_top, vx_bottom, vx_middle, vx_top]
    int stateSize   = 6;
    int measSize    = 3; // We measure x positions at three points
    int controlSize = 0; // No control input

    leftLaneKF.init(stateSize, measSize, controlSize, CV_32F);
    rightLaneKF.init(stateSize, measSize, controlSize, CV_32F);

    // Initialize state transition matrix (constant velocity model)
    // [ 1 0 0 1 0 0 ]
    // [ 0 1 0 0 1 0 ]
    // [ 0 0 1 0 0 1 ]
    // [ 0 0 0 1 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    cv::setIdentity(leftLaneKF.transitionMatrix);
    cv::setIdentity(rightLaneKF.transitionMatrix);
    for (int i = 0; i < 3; i++)
    {
        leftLaneKF.transitionMatrix.at<float>(i, i + 3)  = 1.0f;
        rightLaneKF.transitionMatrix.at<float>(i, i + 3) = 1.0f;
    }

    // Initialize measurement matrix
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 1 0 0 0 ]
    cv::setIdentity(leftLaneKF.measurementMatrix, cv::Scalar(1));
    cv::setIdentity(rightLaneKF.measurementMatrix, cv::Scalar(1));

    // Set process noise covariance
    cv::setIdentity(leftLaneKF.processNoiseCov, cv::Scalar(5e-4));
    cv::setIdentity(rightLaneKF.processNoiseCov, cv::Scalar(5e-4));

    // Set measurement noise covariance
    cv::setIdentity(leftLaneKF.measurementNoiseCov, cv::Scalar(1e-1));
    cv::setIdentity(rightLaneKF.measurementNoiseCov, cv::Scalar(1e-1));

    // Set error covariance
    cv::setIdentity(leftLaneKF.errorCovPost, cv::Scalar(1));
    cv::setIdentity(rightLaneKF.errorCovPost, cv::Scalar(1));

    // Initialize state with current lane positions
    if (!leftCurve.empty() && leftCurve.size() >= 3)
    {
        int bottom_idx = 0, mid_idx = leftCurve.size() / 2,
            top_idx = leftCurve.size() - 1;

        leftLaneKF.statePost.at<float>(0) = leftCurve[bottom_idx].x;
        leftLaneKF.statePost.at<float>(1) = leftCurve[mid_idx].x;
        leftLaneKF.statePost.at<float>(2) = leftCurve[top_idx].x;
        // Initialize velocities as 0
        leftLaneKF.statePost.at<float>(3) = 0;
        leftLaneKF.statePost.at<float>(4) = 0;
        leftLaneKF.statePost.at<float>(5) = 0;
    }

    if (!rightCurve.empty() && rightCurve.size() >= 3)
    {
        int bottom_idx = 0, mid_idx = rightCurve.size() / 2,
            top_idx = rightCurve.size() - 1;

        rightLaneKF.statePost.at<float>(0) = rightCurve[bottom_idx].x;
        rightLaneKF.statePost.at<float>(1) = rightCurve[mid_idx].x;
        rightLaneKF.statePost.at<float>(2) = rightCurve[top_idx].x;
        // Initialize velocities as 0
        rightLaneKF.statePost.at<float>(3) = 0;
        rightLaneKF.statePost.at<float>(4) = 0;
        rightLaneKF.statePost.at<float>(5) = 0;
    }

    kfInitialized = true;
}

void LaneDetectorCV::sendCoefs(const std::vector<cv::Point>& leftCurve,
                               const std::vector<cv::Point>& rightCurve)
{
    // Extract coefficients for the left curve
    cv::Mat leftCoeffs;
    if (!leftCurve.empty() && leftCurve.size() >= 3)
    {
        std::vector<float> x_vals, y_vals;
        for (const auto& pt : leftCurve)
        {
            x_vals.push_back(pt.x);
            y_vals.push_back(pt.y);
        }

        cv::Mat x_mat(x_vals), y_mat(y_vals);
        leftCoeffs = polyfit(y_mat, x_mat, 2);
    }

    // Extract coefficients for the right curve
    cv::Mat rightCoeffs;
    if (!rightCurve.empty() && rightCurve.size() >= 3)
    {
        std::vector<float> x_vals, y_vals;
        for (const auto& pt : rightCurve)
        {
            x_vals.push_back(pt.x);
            y_vals.push_back(pt.y);
        }

        cv::Mat x_mat(x_vals), y_mat(y_vals);
        rightCoeffs = polyfit(y_mat, x_mat, 2);
    }

    // Publish the coefficients if they were successfully calculated
    if (!leftCoeffs.empty() && !rightCoeffs.empty() && leftCoeffs.rows >= 3 &&
        rightCoeffs.rows >= 3)
    {
        // Extract coefficients
        float leftA = static_cast<float>(
            leftCoeffs.at<double>(0)); // quadratic coefficient
        float leftB =
            static_cast<float>(leftCoeffs.at<double>(1)); // linear coefficient
        float leftC =
            static_cast<float>(leftCoeffs.at<double>(2)); // constant term
        std::cout << "Left lane polynomial: " << leftA << "y² + " 
            << leftB << "y + " << leftC << std::endl;
        float rightA = static_cast<float>(
            rightCoeffs.at<double>(0)); // quadratic coefficient
        float rightB =
            static_cast<float>(rightCoeffs.at<double>(1)); // linear coefficient
        float rightC =
            static_cast<float>(rightCoeffs.at<double>(2)); // constant term
        std::cout << "Right lane polynomial: " << rightA << "y² + " 
                << rightB << "y + " << rightC << std::endl;
        // CAN message addresses
        const uint32_t LEFT_LANE_ADDR  = 0x100;
        const uint32_t RIGHT_LANE_ADDR = 0x101;

        // Create buffer for CAN messages (8 bytes per message)
        uint8_t buffer[8];

        // Send null message first
        memset(buffer, 0, sizeof(buffer));
        int32_t coefA = 0;
        int32_t coefB = 1;
        int32_t coefC = 2;
        
        // Send left lane coefficients one at a time
        // Coefficient A
        memcpy(buffer, &coefA, sizeof(int32_t));
        memcpy(buffer +  sizeof(int32_t), &leftA, sizeof(float));
        canBus->writeMessage(LEFT_LANE_ADDR, buffer, sizeof(buffer));

        // Coefficient B
        memcpy(buffer, &coefB, sizeof(int32_t));
        memcpy(buffer +  sizeof(int32_t), &leftB, sizeof(float));
        canBus->writeMessage(LEFT_LANE_ADDR, buffer, sizeof(buffer));

        // Coefficient C
        memcpy(buffer, &coefC, sizeof(int32_t));
        memcpy(buffer +  sizeof(int32_t), &leftC, sizeof(float));
        canBus->writeMessage(LEFT_LANE_ADDR, buffer, sizeof(buffer));

        memset(buffer, 0, sizeof(buffer));

        // Send right lane coefficients one at a time
        // Coefficient A
        memcpy(buffer, &coefA, sizeof(int32_t));
        memcpy(buffer +  sizeof(int32_t), &rightA, sizeof(float));
        canBus->writeMessage(RIGHT_LANE_ADDR, buffer, sizeof(buffer));

        // Coefficient B
        memcpy(buffer, &coefB, sizeof(int32_t));
        memcpy(buffer +  sizeof(int32_t), &rightB, sizeof(float));
        canBus->writeMessage(RIGHT_LANE_ADDR, buffer, sizeof(buffer));

        // Coefficient C
        memcpy(buffer, &coefC, sizeof(int32_t));
        memcpy(buffer +  sizeof(int32_t), &rightC, sizeof(float));
        canBus->writeMessage(RIGHT_LANE_ADDR, buffer, sizeof(buffer));


        // Log the sent coefficients
        std::cout << "Left lane coeffs: " << leftA << ", " << leftB << ", "
                  << leftC << std::endl;
        std::cout << "Right lane coeffs: " << rightA << ", " << rightB << ", "
                  << rightC << std::endl;
    }
}
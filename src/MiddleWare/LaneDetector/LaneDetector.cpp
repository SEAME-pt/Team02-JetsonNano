#include "LaneDetector.hpp"

#include <sys/time.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace zenoh;

LaneDetector::LaneDetector(const std::string& enginePath, const std::string& pipeline,
    std::shared_ptr<zenoh::Session> session)
    : cap(pipeline, cv::CAP_GSTREAMER), session_(session), FRAME_SKIP(8), laneWidthEstimate(0.0), firstFrame(true),
    frame_count(0)
{
    createExecutionContext(enginePath);

    // Set highest stream priority
    int leastPriority, greatestPriority;
    cudaDeviceGetStreamPriorityRange(&leastPriority, &greatestPriority);
    cudaStreamCreateWithPriority(&stream, cudaStreamNonBlocking,
                                 greatestPriority);

    // Pin memory for faster transfers
    void* input_ptr;
    void* output_ptr;
    cudaHostAlloc(&input_ptr, 3 * 128 * 256 * sizeof(float),
                  cudaHostAllocMapped);
    cudaHostAlloc(&output_ptr, 2 * 128 * 256 * sizeof(float),
                  cudaHostAllocMapped);
    inputData  = static_cast<float*>(input_ptr);
    outputData = static_cast<float*>(output_ptr);

    // Allocate GPU memory
    size_t pitch;
    cudaMallocPitch(&inputDevice, &pitch, 256 * sizeof(float), 128 * 3);
    cudaMallocPitch(&outputDevice, &pitch, 256 * sizeof(float), 128 * 2);

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
}

LaneDetector::~LaneDetector()
{
    cudaFreeHost(inputData);
    cudaFreeHost(outputData);
    cudaFree(inputDevice);
    cudaFree(outputDevice);
    cudaStreamDestroy(stream);
}

void LaneDetector::setCalibrationParameters(void)
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

Mat LaneDetector::regionOfInterest(const Mat& img,
    const vector<Point>& vertices)
{
    Mat mask = Mat::zeros(img.size(), img.type());
    fillPoly(mask, vector<vector<Point>>{vertices}, Scalar(255));
    Mat masked;
    bitwise_and(img, mask, masked);
    return masked;
}

// Polynomial fitting using OpenCV
Mat LaneDetector::polyfit(const Mat& y_vals, const Mat& x_vals, int degree)
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

double LaneDetector::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

void LaneDetector::detect(cv::Mat& frame)
{
    static cudaEvent_t start, stop; // Make events static
    static bool eventsCreated = false;
    // static cudaGraph_t graph;
    // static cudaGraphExec_t graphExec;
    // static bool graphCreated = false;

    if (!eventsCreated)
    {
        cudaEventCreate(&start);
        cudaEventCreate(&stop);
        eventsCreated = true;
    }

    cudaEventRecord(start, stream);

    // Preprocess
    preProcess(frame);

    // Copy to GPU
    cudaMemcpyAsync(inputDevice, inputData, 3 * 128 * 256 * sizeof(float),
                    cudaMemcpyHostToDevice, stream);

    // Run inference with optimization flags
    void* bindings[] = {inputDevice, outputDevice};
    context->enqueueV2(bindings, stream, nullptr);

    // Copy back to CPU
    cudaMemcpyAsync(outputData, outputDevice, 2 * 128 * 256 * sizeof(float),
                    cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

    // Postprocess
    postProcess(frame);

    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);

    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Inference time: " << milliseconds << "ms\n";
}

void LaneDetector::run()
{
    bool mapsInitialized = false;
    kfInitialized = false;

    // Initialize the lane detector publisher
    
    cv::Mat frame;
    
    // Set camera buffer size
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    publisher_ = std::make_shared<LaneDetectorPublisher>(session_);

    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;
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
            cv::Mat undistorted;
            remap(frame, undistorted, map1, map2, INTER_LINEAR);
            frame = undistorted;
            detect(frame);
            imshow("Lane Detection", frame);
        }
        frame_count++;

        if (cv::waitKey(1) == 'q')
            break;
    }

    cv::destroyAllWindows();
}

void LaneDetector::preProcess(const cv::Mat& frame)
{
    static cv::Mat resized(128, 256, CV_8UC3);
    static cv::Mat float_mat(128, 256, CV_32FC3);

    // Use INTER_NEAREST for faster resizing
    cv::resize(frame, resized, cv::Size(256, 128), 0, 0, cv::INTER_NEAREST);

    // Optimize memory access pattern
    const int plane_size      = 128 * 256;
    const uint8_t* frame_data = resized.data;

    // Set number of threads for OpenMP
    omp_set_num_threads(4);

// Use collapse(2) to better parallelize nested loops
    #pragma omp parallel for collapse(2) schedule(static)
    for (int c = 0; c < 3; c++)
    {
        for (int i = 0; i < plane_size; i++)
        {
            // Direct memory access optimization
            inputData[c * plane_size + i] = frame_data[i * 3 + c] / 255.0f;
        }
    }
}

void LaneDetector::postProcess(cv::Mat& frame)
{
    static cv::Mat mask(128, 256, CV_8UC1);
    static cv::Mat resized_mask;
    static cv::Mat colored_mask;

    uchar* mask_data       = mask.data;
    const int total_pixels = 128 * 256;

#pragma omp parallel for
    for (int i = 0; i < total_pixels; i++)
    {
        float p0     = outputData[i];
        float p1     = outputData[total_pixels + i];
        mask_data[i] = (p1 > p0) ? 255 : 0;
    }

    // Collect points in mask coordinates
    std::vector<cv::Point> maskPoints;
    for (int y = 0; y < mask.rows; y++) {
        for (int x = 0; x < mask.cols; x++) {
            if (mask.at<uchar>(y, x) == 255) {
                maskPoints.push_back(cv::Point(x, y));
            }
        }
    }
    
    // Scale all points to frame coordinates
    std::vector<cv::Point> lanePoints;
    lanePoints.reserve(maskPoints.size()); // Pre-allocate for performance
    float x_scale = static_cast<float>(frame.cols) / mask.cols;
    float y_scale = static_cast<float>(frame.rows) / mask.rows;
    
    for (const auto& pt : maskPoints) {
        int scaledX = pt.x * x_scale;
        int scaledY = pt.y * y_scale;
        lanePoints.push_back(cv::Point(scaledX, scaledY));
    }
    cv::resize(mask, resized_mask, frame.size(), 0, 0, cv::INTER_NEAREST);
    cv::cvtColor(resized_mask, colored_mask, cv::COLOR_GRAY2BGR);
    createLanes(lanePoints, frame);

}

void LaneDetector::createLanes(std::vector<cv::Point> lanePoints, cv::Mat& frame)
{
    // Initialize lane width estimate on first frame.
    if (firstFrame)
    {
        laneWidthEstimate = frame.cols * 0.45;
        firstFrame = false;
    }

    //Define Left and Right lanes
    // 1. Cluster points using Mean Shift
    std::vector<cv::Point> leftPoints, rightPoints;
    clusterLanePoints(lanePoints, leftPoints, rightPoints, frame);
    
    // 2. Fit polynomial curves to each set of points
    std::vector<cv::Point> leftCurve = fitCurveToPoints(leftPoints, frame);
    std::vector<cv::Point> rightCurve = fitCurveToPoints(rightPoints, frame);
    
    // 3. Apply Kalman filtering for temporal smoothing and prediction
    // Initialize Kalman filter if this is the first good detection
    if (!kfInitialized && !leftCurve.empty() && !rightCurve.empty()) {
        initKalmanFilters(leftCurve, rightCurve);
    }
    
    // Use Kalman filter predictions when lanes disappear or are unstable
    if (kfInitialized) {
        // Always predict next state
        cv::Mat leftPredicted = leftLaneKF.predict();
        cv::Mat rightPredicted = rightLaneKF.predict();
        
        // If we have actual measurements, use them to update the filter
        if (!leftCurve.empty() && leftCurve.size() >= 3) {
            int bottom_idx = 0;
            int mid_idx = leftCurve.size() / 2;
            int top_idx = leftCurve.size() - 1;
            
            cv::Mat measurement = (cv::Mat_<float>(3, 1) << 
                leftCurve[bottom_idx].x,
                leftCurve[mid_idx].x, 
                leftCurve[top_idx].x);
                
            leftLaneKF.correct(measurement);
        } 
        else if (leftCurve.empty() || leftCurve.size() < 3) {
            // No valid left lane detected - use prediction
            std::vector<cv::Point> predictedLeftCurve;
            int height = frame.rows;
            
            if (!rightCurve.empty() && rightCurve.size() >= 3) {
                // Use right lane with lane width offset (more accurate)
                int bottom_idx = 0;
                int mid_idx = rightCurve.size() / 2;
                int top_idx = rightCurve.size() - 1;
                
                // Fit polynomial to right lane to get curvature
                std::vector<cv::Point2f> rightPoints = {
                    cv::Point2f(rightCurve[bottom_idx].x, rightCurve[bottom_idx].y),
                    cv::Point2f(rightCurve[mid_idx].x, rightCurve[mid_idx].y),
                    cv::Point2f(rightCurve[top_idx].x, rightCurve[top_idx].y)
                };
                
                std::vector<float> x_vals, y_vals;
                for (auto& pt : rightPoints) {
                    x_vals.push_back(pt.x);
                    y_vals.push_back(pt.y);
                }
                
                cv::Mat x_mat(x_vals), y_mat(y_vals);
                cv::Mat rightCoeffs = polyfit(y_mat, x_mat, 2);
                
                // Generate left curve by shifting right curve left by lane width
                if (!rightCoeffs.empty() && rightCoeffs.rows >= 3) {
                    double a = rightCoeffs.at<double>(0); // quadratic term (curvature)
                    double b = rightCoeffs.at<double>(1); // linear term (slope)
                    double c = rightCoeffs.at<double>(2) - laneWidthEstimate; // constant (x-offset)
                    
                    for (int y = height; y >= height / 3; y -= 5) {
                        double x = a * y * y + b * y + c;
                        predictedLeftCurve.push_back(cv::Point(round(x), y));
                    }
                    
                    leftCurve = predictedLeftCurve;
                }
            }
            else {
                // No right lane either - use pure Kalman prediction
                float bottom_x = leftPredicted.at<float>(0);
                float mid_x = leftPredicted.at<float>(1);
                float top_x = leftPredicted.at<float>(2);
                
                // Generate curve points using quadratic interpolation
                int bottom_y = height;
                int mid_y = height / 2;
                int top_y = height / 3;
                
                cv::Mat Y = (cv::Mat_<double>(3, 1) << bottom_y, mid_y, top_y);
                cv::Mat X = (cv::Mat_<double>(3, 1) << bottom_x, mid_x, top_x);
                cv::Mat coeffs = polyfit(Y, X, 2);
                
                if (!coeffs.empty() && coeffs.rows >= 3) {
                    for (int y = height; y >= height / 3; y -= 5) {
                        double x = coeffs.at<double>(0) * y * y + 
                                  coeffs.at<double>(1) * y + 
                                  coeffs.at<double>(2);
                        predictedLeftCurve.push_back(cv::Point(round(x), y));
                    }
                    leftCurve = predictedLeftCurve;
                }
            }
        }
        
        // Similar logic for right curve
        if (!rightCurve.empty() && rightCurve.size() >= 3) {
            int bottom_idx = 0;
            int mid_idx = rightCurve.size() / 2;
            int top_idx = rightCurve.size() - 1;
            
            cv::Mat measurement = (cv::Mat_<float>(3, 1) << 
                rightCurve[bottom_idx].x,
                rightCurve[mid_idx].x, 
                rightCurve[top_idx].x);
                
            rightLaneKF.correct(measurement);
        }
        else if (rightCurve.empty() || rightCurve.size() < 3) {
            // No valid right lane detected - use prediction
            std::vector<cv::Point> predictedRightCurve;
            int height = frame.rows;
            
            if (!leftCurve.empty() && leftCurve.size() >= 3) {
                // Use left lane with lane width offset (more accurate)
                int bottom_idx = 0;
                int mid_idx = leftCurve.size() / 2;
                int top_idx = leftCurve.size() - 1;
                
                // Fit polynomial to left lane to get curvature
                std::vector<cv::Point2f> leftPoints = {
                    cv::Point2f(leftCurve[bottom_idx].x, leftCurve[bottom_idx].y),
                    cv::Point2f(leftCurve[mid_idx].x, leftCurve[mid_idx].y),
                    cv::Point2f(leftCurve[top_idx].x, leftCurve[top_idx].y)
                };
                
                std::vector<float> x_vals, y_vals;
                for (auto& pt : leftPoints) {
                    x_vals.push_back(pt.x);
                    y_vals.push_back(pt.y);
                }
                
                cv::Mat x_mat(x_vals), y_mat(y_vals);
                cv::Mat leftCoeffs = polyfit(y_mat, x_mat, 2);
                
                // Generate right curve by shifting left curve right by lane width
                if (!leftCoeffs.empty() && leftCoeffs.rows >= 3) {
                    double a = leftCoeffs.at<double>(0); // quadratic term (curvature)
                    double b = leftCoeffs.at<double>(1); // linear term (slope)
                    double c = leftCoeffs.at<double>(2) + laneWidthEstimate; // constant (x-offset)
                    
                    for (int y = height; y >= height / 3; y -= 5) {
                        double x = a * y * y + b * y + c;
                        predictedRightCurve.push_back(cv::Point(round(x), y));
                    }
                    
                    rightCurve = predictedRightCurve;
                }
            }
            else {
                // No left lane either - use pure Kalman prediction
                float bottom_x = rightPredicted.at<float>(0);
                float mid_x = rightPredicted.at<float>(1);
                float top_x = rightPredicted.at<float>(2);
                
                // Generate curve points using quadratic interpolation
                int bottom_y = height;
                int mid_y = height / 2;
                int top_y = height / 3;
                
                cv::Mat Y = (cv::Mat_<double>(3, 1) << bottom_y, mid_y, top_y);
                cv::Mat X = (cv::Mat_<double>(3, 1) << bottom_x, mid_x, top_x);
                cv::Mat coeffs = polyfit(Y, X, 2);
                
                if (!coeffs.empty() && coeffs.rows >= 3) {
                    for (int y = height; y >= height / 3; y -= 5) {
                        double x = coeffs.at<double>(0) * y * y + 
                                  coeffs.at<double>(1) * y + 
                                  coeffs.at<double>(2);
                        predictedRightCurve.push_back(cv::Point(round(x), y));
                    }
                    rightCurve = predictedRightCurve;
                }
            }
        }
    }
    
    // Update lane width estimate when both curves are detected
    if (!leftCurve.empty() && !rightCurve.empty()) {
        int bottomY = frame.rows - 1;
        int leftX = -1, rightX = -1;
        
        // Find points near the bottom of the image
        for (const auto& pt : leftCurve) {
            if (pt.y == bottomY || (leftX == -1 && pt.y > frame.rows * 0.7)) {
                leftX = pt.x;
                break;
            }
        }
        
        for (const auto& pt : rightCurve) {
            if (pt.y == bottomY || (rightX == -1 && pt.y > frame.rows * 0.7)) {
                rightX = pt.x;
                break;
            }
        }
        
        if (leftX != -1 && rightX != -1) {
            double currentWidth = rightX - leftX;
            // Exponential moving average to smooth lane width estimate
            laneWidthEstimate = 0.2 * currentWidth + 0.8 * laneWidthEstimate;
        }
    }


    // 4. Draw the detected lanes
    drawLanes(frame, leftCurve, rightCurve);

}

void LaneDetector::drawLanes(cv::Mat& frame, 
                            const std::vector<cv::Point>& leftCurve, 
                            const std::vector<cv::Point>& rightCurve)
{
    // Draw left lane in blue
    if (!leftCurve.empty() && leftCurve.size() >= 2) {
        for (size_t i = 1; i < leftCurve.size(); i++) {
            cv::line(frame, leftCurve[i-1], leftCurve[i], cv::Scalar(255, 0, 0), 3);
        }
    }
    
    // Draw right lane in green
    if (!rightCurve.empty() && rightCurve.size() >= 2) {
        for (size_t i = 1; i < rightCurve.size(); i++) {
            cv::line(frame, rightCurve[i-1], rightCurve[i], cv::Scalar(0, 255, 0), 3);
        }
    }
}

void LaneDetector::clusterLanePoints(const std::vector<cv::Point>& points, 
                                     std::vector<cv::Point>& leftPoints,
                                     std::vector<cv::Point>& rightPoints,
                                    cv::Mat& frame)
{
    if (points.empty()) {
        return;
    }
    
    // Convert points to the format expected by meanShift
    std::vector<cv::Point2f> samples;
    for (const auto& pt : points) {
        samples.push_back(cv::Point2f(pt.x, pt.y));
    }
    
    // Convert to Mat format (required for clustering algorithms)
    cv::Mat pointsMat(samples.size(), 2, CV_32F);
    for (size_t i = 0; i < samples.size(); i++) {
        pointsMat.at<float>(i, 0) = samples[i].x;
        pointsMat.at<float>(i, 1) = samples[i].y;
    }
    
    // Set meanShift parameters
    //const float spatialRadius = frame.cols * 0.2; // 20% of image width
    //const float colorRadius = 1.0; // Not used for spatial-only clustering
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 1.0);
    
    // Apply meanShift clustering
    cv::Mat labels, centers;
    int k = 2; // Two clusters (left and right lane)
    cv::kmeans(pointsMat, k, labels, 
            criteria, 3, cv::KMEANS_PP_CENTERS, centers);
    int clusterCount = k;
    // If only one cluster is found, use x-coordinate to separate
    if (clusterCount <= 1) {
        // Fallback to simple X-coordinate based separation
        int midX = frame.cols / 2;
        for (const auto& pt : points) {
            if (pt.x < midX) {
                leftPoints.push_back(pt);
            } else {
                rightPoints.push_back(pt);
            }
        }
        return;
    }
    
    // Group points by their cluster labels
    std::map<int, std::vector<cv::Point>> clusters;
    for (int i = 0; i < labels.rows; i++) {
        int label = labels.at<int>(i, 0);
        clusters[label].push_back(points[i]);
    }
    
    // Find the leftmost and rightmost clusters
    int leftLabel = -1, rightLabel = -1;
    float leftMeanX = frame.cols, rightMeanX = 0;
    
    for (const auto& pair : clusters) {
        // Calculate mean X position for this cluster
        float sumX = 0;
        for (const auto& pt : pair.second) {
            sumX += pt.x;
        }
        float meanX = sumX / pair.second.size();
        
        // Update leftmost and rightmost clusters
        if (meanX < leftMeanX) {
            leftMeanX = meanX;
            leftLabel = pair.first;
        }
        if (meanX > rightMeanX) {
            rightMeanX = meanX;
            rightLabel = pair.first;
        }
    }
    
    // Assign points to left and right
    if (leftLabel >= 0) {
        leftPoints = clusters[leftLabel];
    }
    if (rightLabel >= 0) {
        rightPoints = clusters[rightLabel];
    }
    
    // If the clusters are suspiciously close, verify using time-based consistency
    float clusterDistance = rightMeanX - leftMeanX;
    if (clusterDistance < laneWidthEstimate * 0.5 && !prevLeftPoints.empty() && !prevRightPoints.empty()) {
        // Use previous frame's classification to help resolve ambiguity
        reassignPointsUsingPreviousFrame(leftPoints, rightPoints);
    }
    
    // Save for next frame
    prevLeftPoints = leftPoints;
    prevRightPoints = rightPoints;
}

std::vector<cv::Point> LaneDetector::fitCurveToPoints(const std::vector<cv::Point>& points, cv::Mat& frame)
{
    if (points.size() < 3) {
        return std::vector<cv::Point>();
    }
    
    // Convert to vectors for polynomial fitting
    std::vector<float> x_vals, y_vals;
    for (const auto& pt : points) {
        x_vals.push_back(pt.x);
        y_vals.push_back(pt.y);
    }
    
    // Convert to Mat for polyfit
    cv::Mat x_mat(x_vals), y_mat(y_vals);
    int degree = 2; // Quadratic curve
    cv::Mat coeffs = polyfit(y_mat, x_mat, degree);
    
    // Generate smooth curve
    std::vector<cv::Point> curve;
    if (!coeffs.empty() && coeffs.rows >= 3) {
        for (int y = frame.rows; y >= frame.rows/2; y -= 5) {
            double x = coeffs.at<double>(0) * y * y + 
                      coeffs.at<double>(1) * y + 
                      coeffs.at<double>(2);
            
            if (!std::isnan(x) && !std::isinf(x)) {
                curve.push_back(cv::Point(round(x), y));
            }
        }
    }
    
    return curve;
}

void LaneDetector::reassignPointsUsingPreviousFrame(std::vector<cv::Point>& leftPoints, 
                                                   std::vector<cv::Point>& rightPoints)
{
    // Temporary storage for potentially reassigned points
    std::vector<cv::Point> newLeftPoints, newRightPoints;
    
    // Combine all points for reassessment
    std::vector<cv::Point> allPoints;
    allPoints.insert(allPoints.end(), leftPoints.begin(), leftPoints.end());
    allPoints.insert(allPoints.end(), rightPoints.begin(), rightPoints.end());
    
    // For each point, find the closest point from previous left and right sets
    for (const auto& pt : allPoints) {
        double minDistLeft = std::numeric_limits<double>::max();
        double minDistRight = std::numeric_limits<double>::max();
        
        // Find minimum distance to previous left points
        for (const auto& prevPt : prevLeftPoints) {
            double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) + std::pow(pt.y - prevPt.y, 2));
            minDistLeft = std::min(minDistLeft, dist);
        }
        
        // Find minimum distance to previous right points
        for (const auto& prevPt : prevRightPoints) {
            double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) + std::pow(pt.y - prevPt.y, 2));
            minDistRight = std::min(minDistRight, dist);
        }
        
        // Assign to the lane with the closest previous point
        if (minDistLeft < minDistRight) {
            newLeftPoints.push_back(pt);
        } else {
            newRightPoints.push_back(pt);
        }
    }
    
    // Update with reassigned points if we have enough in each group
    if (newLeftPoints.size() >= 3 && newRightPoints.size() >= 3) {
        leftPoints = newLeftPoints;
        rightPoints = newRightPoints;
    }
}


void LaneDetector::createExecutionContext(const std::string& enginePath)
{
    std::ifstream file(enginePath, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Failed to open engine file");
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> engineData(size);
    file.read(engineData.data(), size);

    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(logger);
    nvinfer1::ICudaEngine* engine =
        runtime->deserializeCudaEngine(engineData.data(), size);
    context.reset(engine->createExecutionContext());
}


void LaneDetector::initKalmanFilters(const std::vector<cv::Point>& leftCurve, 
                                   const std::vector<cv::Point>& rightCurve)
{
    // Track 3 key points (bottom, middle, top) of each lane
    // State: [x_bottom, x_middle, x_top, vx_bottom, vx_middle, vx_top]
    int stateSize = 6;
    int measSize = 3;  // We measure x positions at three points
    int controlSize = 0;  // No control input
    
    leftLaneKF.init(stateSize, measSize, controlSize, CV_32F);
    rightLaneKF.init(stateSize, measSize, controlSize, CV_32F);
    
    // Initialize state transition matrix (constant velocity model)
    cv::setIdentity(leftLaneKF.transitionMatrix);
    cv::setIdentity(rightLaneKF.transitionMatrix);
    for (int i = 0; i < 3; i++) {
        leftLaneKF.transitionMatrix.at<float>(i, i + 3) = 1.0f;
        rightLaneKF.transitionMatrix.at<float>(i, i + 3) = 1.0f;
    }
    
    // Initialize measurement matrix
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
    if (!leftCurve.empty() && leftCurve.size() >= 3) {
        int bottom_idx = 0;
        int mid_idx = leftCurve.size() / 2;
        int top_idx = leftCurve.size() - 1;
        
        leftLaneKF.statePost.at<float>(0) = leftCurve[bottom_idx].x;
        leftLaneKF.statePost.at<float>(1) = leftCurve[mid_idx].x;
        leftLaneKF.statePost.at<float>(2) = leftCurve[top_idx].x;
        // Initialize velocities as 0
        leftLaneKF.statePost.at<float>(3) = 0;
        leftLaneKF.statePost.at<float>(4) = 0;
        leftLaneKF.statePost.at<float>(5) = 0;
    }
    
    if (!rightCurve.empty() && rightCurve.size() >= 3) {
        int bottom_idx = 0;
        int mid_idx = rightCurve.size() / 2;
        int top_idx = rightCurve.size() - 1;
        
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
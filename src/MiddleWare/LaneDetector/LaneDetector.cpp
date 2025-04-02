#include "LaneDetector.hpp"

#include <sys/time.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace zenoh;

Logger logger;

LaneDetector::LaneDetector(const std::string& enginePath, const std::string& pipeline,
    std::shared_ptr<zenoh::Session> session)
    : cap(pipeline, cv::CAP_GSTREAMER), session_(session), FRAME_SKIP(4), laneWidthEstimate(0.0), firstFrame(true),
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
    cudaHostAlloc(&input_ptr, 3 * HEIGHT * WIDTH * sizeof(float),
                  cudaHostAllocMapped);
    cudaHostAlloc(&output_ptr, 2 * HEIGHT * WIDTH * sizeof(float),
                  cudaHostAllocMapped);
    inputData  = static_cast<float*>(input_ptr);
    outputData = static_cast<float*>(output_ptr);

    // Allocate GPU memory
    size_t pitch;
    cudaMallocPitch(&inputDevice, &pitch, WIDTH * sizeof(float), HEIGHT * 3);
    cudaMallocPitch(&outputDevice, &pitch, WIDTH * sizeof(float), HEIGHT * 2);

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
    cudaMemcpyAsync(inputDevice, inputData, 3 * HEIGHT * WIDTH * sizeof(float),
                    cudaMemcpyHostToDevice, stream);

    // Run inference with optimization flags
    void* bindings[] = {inputDevice, outputDevice};
    context->enqueueV2(bindings, stream, nullptr);

    // Copy back to CPU
    cudaMemcpyAsync(outputData, outputDevice, 2 * HEIGHT * WIDTH * sizeof(float),
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
            //imshow("Lane Detection", frame);
            publisher_->publishCameraFrame(frame);

        }
        frame_count++;

        if (cv::waitKey(1) == 'q')
            break;
    }

    cv::destroyAllWindows();
}

void LaneDetector::preProcess(const cv::Mat& frame)
{
    static cv::Mat resized(HEIGHT, WIDTH, CV_8UC3);
    static cv::Mat float_mat(HEIGHT, WIDTH, CV_32FC3);

    // Use INTER_NEAREST for faster resizing
    cv::resize(frame, resized, cv::Size(WIDTH, HEIGHT), 0, 0, cv::INTER_NEAREST);

    // Optimize memory access pattern
    const int plane_size      = HEIGHT * WIDTH;
    const uint8_t* frame_data = resized.data;

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
    static cv::Mat mask(HEIGHT, WIDTH, CV_8UC1);
    static cv::Mat resized_mask;
    static cv::Mat colored_mask;

    uchar* mask_data       = mask.data;
    const int total_pixels = HEIGHT * WIDTH;

    for (int i = 0; i < total_pixels; i++)
    {
        float p0     = outputData[i];
        float p1     = outputData[total_pixels + i];
        mask_data[i] = (p0 > p1) ? 255 : 0;
    }

    // // Resize the mask to match frame size
    // cv::resize(mask, resized_mask, frame.size(), 0, 0, cv::INTER_NEAREST);
    
    // // Create a color overlay - convert mask to a colored version
    // cv::Mat overlay = cv::Mat::zeros(frame.size(), frame.type());
    // // Make the mask areas blue with some transparency
    // overlay.setTo(cv::Scalar(120, 0, 0), resized_mask);
    
    // // Blend with original image using weighted addition
    // cv::addWeighted(frame, 1.0, overlay, 0.5, 0, frame);

    
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
    // for (const auto& pt : lanePoints) {
    //     cv::circle(frame, pt, 2, cv::Scalar(255, 255, 255), -1); // White for all points
    // }

    if (firstFrame)
    {
        laneWidthEstimate = frame.cols * 0.45;
        firstFrame = false;
    }

    // Define Left and Right lanes
    // 1. Cluster points using Mean Shift
    std::vector<cv::Point> leftPoints, rightPoints;
    clusterLanePoints(lanePoints, leftPoints, rightPoints, frame);
    
    // 2. Fit polynomial curves to each set of points
    std::vector<cv::Point> leftCurve = fitCurveToPoints(leftPoints, frame);
    std::vector<cv::Point> rightCurve = fitCurveToPoints(rightPoints, frame);
    
    // // 3. Apply Kalman filtering for temporal smoothing and prediction
    // // Initialize Kalman filter if this is the first good detection
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
                std::vector<cv::Point> averagedLeftCurve = leftCurve;

                // For each point in the curve
                for (size_t i = 0; i < leftCurve.size(); i++)
                {
                    int sumX = 0, sumY = 0;
                    double totalWeight = 0;

                    // Average across history (weighted, with recent frames having more weight)
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
                        averagedLeftCurve[i].x = static_cast<int>(sumX / totalWeight);
                        averagedLeftCurve[i].y = static_cast<int>(sumY / totalWeight);
                    }
                }
                leftCurve = averagedLeftCurve;
            }

            // Dynamic alpha smoothing based on curvature
            if (!prevLeftCurve.empty() && prevLeftCurve.size() == leftCurve.size())
            {
                double curvature = 0;
                if (leftCurve.size() >= 3)
                {
                    // Calculate curvature using all points
                    std::vector<float> x_vals, y_vals;
                    for (const auto& pt : leftCurve)
                    {
                        x_vals.push_back(pt.x);
                        y_vals.push_back(pt.y);
                    }

                    cv::Mat x_mat(x_vals), y_mat(y_vals);
                    cv::Mat coeffs = polyfit(y_mat, x_mat, 2);
                    if (!coeffs.empty() && coeffs.rows >= 3)
                    {
                        curvature = std::abs(coeffs.at<double>(0));
                    }
                }

                // Lower alpha for smoother transitions with higher curvature
                double dynamicAlpha = std::min(0.4, alpha + curvature * 1000);

                for (size_t i = 0; i < leftCurve.size(); i++)
                {
                    leftCurve[i].x = static_cast<int>(dynamicAlpha * leftCurve[i].x + 
                                                      (1 - dynamicAlpha) * prevLeftCurve[i].x);
                    leftCurve[i].y = static_cast<int>(dynamicAlpha * leftCurve[i].y + 
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
                std::vector<cv::Point> averagedRightCurve = rightCurve;

                // For each point in the curve
                for (size_t i = 0; i < rightCurve.size(); i++)
                {
                    int sumX = 0, sumY = 0;
                    double totalWeight = 0;

                    // Average across history (weighted, with recent frames having more weight)
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
                        averagedRightCurve[i].x = static_cast<int>(sumX / totalWeight);
                        averagedRightCurve[i].y = static_cast<int>(sumY / totalWeight);
                    }
                }
                rightCurve = averagedRightCurve;
            }

            // Dynamic alpha smoothing based on curvature
            if (!prevRightCurve.empty() && prevRightCurve.size() == rightCurve.size())
            {
                double curvature = 0;
                if (rightCurve.size() >= 3)
                {
                    // Calculate curvature using all points
                    std::vector<float> x_vals, y_vals;
                    for (const auto& pt : rightCurve)
                    {
                        x_vals.push_back(pt.x);
                        y_vals.push_back(pt.y);
                    }

                    cv::Mat x_mat(x_vals), y_mat(y_vals);
                    cv::Mat coeffs = polyfit(y_mat, x_mat, 2);
                    if (!coeffs.empty() && coeffs.rows >= 3)
                    {
                        curvature = std::abs(coeffs.at<double>(0));
                    }
                }

                // Lower alpha for smoother transitions with higher curvature
                double dynamicAlpha = std::min(0.4, alpha + curvature * 1000);

                for (size_t i = 0; i < rightCurve.size(); i++)
                {
                    rightCurve[i].x = static_cast<int>(dynamicAlpha * rightCurve[i].x + 
                                                    (1 - dynamicAlpha) * prevRightCurve[i].x);
                    rightCurve[i].y = static_cast<int>(dynamicAlpha * rightCurve[i].y + 
                                                    (1 - dynamicAlpha) * prevRightCurve[i].y);
                }
            }
        }
        
    }

    // Save current curves for next frame
    prevLeftCurve = leftCurve;
    prevRightCurve = rightCurve;

    // 8. Compute center lane as average of left and right lanes
    std::vector<cv::Point> midCurve;
    if (!leftCurve.empty() && !rightCurve.empty())
    {
        // Make sure we have equal length curves by resampling if needed
        int numPoints = std::min(leftCurve.size(), rightCurve.size());
        for (int i = 0; i < numPoints; i++)
        {
            size_t leftIdx = i * leftCurve.size() / numPoints;
            size_t rightIdx = i * rightCurve.size() / numPoints;
            
            int midX = (leftCurve[leftIdx].x + rightCurve[rightIdx].x) / 2;
            int midY = (leftCurve[leftIdx].y + rightCurve[rightIdx].y) / 2;
            midCurve.push_back(cv::Point(midX, midY));
        }
    }

 // 9. Calculate reference point for lateral error - IMPROVED HANDLING
    cv::Point midPoint;
    int height = frame.rows;
    int width = frame.cols;
    
    if (!midCurve.empty())
    {
        int targetY = height - (2 * height / 3); // 2/3 up from bottom

        // Find closest point to target Y
        size_t closestIdx = 0;
        int minDistance = std::abs(midCurve[0].y - targetY);

        for (size_t i = 1; i < midCurve.size(); i++)
        {
            int distance = std::abs(midCurve[i].y - targetY);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIdx = i;
            }
        }

        // Use the point at found index
        midPoint = midCurve[closestIdx];

        // Sanity check against convergence - verify lane separation at this height
        bool lanesConvergedIncorrectly = false;
        
        if (!leftCurve.empty() && !rightCurve.empty()) {
            // Get left and right lane positions at the same height
            cv::Point leftPos, rightPos;
            for (const auto& pt : leftCurve) {
                if (abs(pt.y - targetY) < 20) {
                    leftPos = pt;
                    break;
                }
            }
            
            for (const auto& pt : rightCurve) {
                if (abs(pt.y - targetY) < 20) {
                    rightPos = pt;
                    break;
                }
            }
            
            // Check if lanes are too close together
            if (leftPos.x != 0 && rightPos.x != 0) {
                float laneDistance = rightPos.x - leftPos.x;
                // If lanes are too close or crossed, something is wrong
                if (laneDistance < laneWidthEstimate * 0.7 || leftPos.x > rightPos.x) {
                    lanesConvergedIncorrectly = true;
                }
            }
        }

        // // Apply temporal smoothing to midPoint with fallback
        // if (prevMidPoint.x != -1 && prevMidPoint.y != -1)
        // {
        //     // Use higher smoothing (more history weight) when there are issues
        //     float gamma = lanesConvergedIncorrectly ? 0.9 : 0.8; // Increased from 0.8
            
        //     // If detected lanes are bad, rely more on history and bias toward center
        //     if (lanesConvergedIncorrectly) {
        //         // Bias toward center on lane convergence issues
        //         float centerBias = 0.2;
        //         midPoint.x = static_cast<int>(gamma * prevMidPoint.x + (1 - gamma) * 
        //                    ((1-centerBias) * midPoint.x + centerBias * (width/2)));
        //     } else {
        //         midPoint.x = static_cast<int>(gamma * prevMidPoint.x + (1 - gamma) * midPoint.x);
        //     }
        //     midPoint.y = static_cast<int>(gamma * prevMidPoint.y + (1 - gamma) * midPoint.y);
        // }

        if (prevMidPoint.x != -1 && prevMidPoint.y != -1)
        {
            // Use LOWER smoothing values for faster response
            float gamma = lanesConvergedIncorrectly ? 0.7 : 0.5; // Reduced from 0.9/0.8
            
            if (lanesConvergedIncorrectly) {
                float centerBias = 0.3; // Increased center bias when lanes converge
                midPoint.x = static_cast<int>(gamma * prevMidPoint.x + (1 - gamma) * 
                        ((1-centerBias) * midPoint.x + centerBias * (width/2)));
            } else {
                midPoint.x = static_cast<int>(gamma * prevMidPoint.x + (1 - gamma) * midPoint.x);
            }
            midPoint.y = static_cast<int>(gamma * prevMidPoint.y + (1 - gamma) * midPoint.y);
        }

        // Visual indicator of target Y
        cv::line(frame, cv::Point(0, targetY), cv::Point(width, targetY), 
                 cv::Scalar(0, 255, 255), 1);
    }
    else
    {
        // IMPROVED FALLBACK: smarter handling when no midCurve is detected
        if (prevMidPoint.x != -1 && prevMidPoint.y != -1) {
            // Check if we're likely in a curve by examining previous lane curves
            bool inCurve = false;
            float curveDirection = 0.0f; // -1 = left curve, +1 = right curve
            
            // Use previous lane curves to determine if we're in a curve
            if (!prevLeftCurve.empty() && prevLeftCurve.size() >= 3) {
                // Sample 3 points from the curve to estimate curvature
                cv::Point top = prevLeftCurve[prevLeftCurve.size()-1];
                cv::Point middle = prevLeftCurve[prevLeftCurve.size()/2];
                cv::Point bottom = prevLeftCurve[0];
                
                // Calculate basic curvature
                float dx1 = middle.x - bottom.x;
                float dx2 = top.x - middle.x;
                
                // If there's a significant change in direction, we're in a curve
                if (abs(dx2 - dx1) > width * 0.03) {
                    inCurve = true;
                    curveDirection = (dx2 > dx1) ? 1.0f : -1.0f;
                }
            }
            
            // Use right curve as backup if left doesn't show a curve
            if (!inCurve && !prevRightCurve.empty() && prevRightCurve.size() >= 3) {
                cv::Point top = prevRightCurve[prevRightCurve.size()-1];
                cv::Point middle = prevRightCurve[prevRightCurve.size()/2];
                cv::Point bottom = prevRightCurve[0];
                
                float dx1 = middle.x - bottom.x;
                float dx2 = top.x - middle.x;
                
                if (abs(dx2 - dx1) > width * 0.03) {
                    inCurve = true;
                    curveDirection = (dx2 > dx1) ? 1.0f : -1.0f;
                }
            }
            
            // Different handling for curves vs. straight sections
            if (inCurve) {
                // In a curve: maintain curve trajectory with slower decay
                float curveDecay = 0.85f; // Slow decay in curves
                int errorReducedX = width/2 + (prevMidPoint.x - width/2) * curveDecay;
                
                // Add a slight bias in the direction of the curve
                errorReducedX += curveDirection * width * 0.01;
                
                midPoint = cv::Point(errorReducedX, height * 2 / 3);
            } else {
                // Straight section: faster return to center
                float straightDecay = 0.7f; // Faster decay when straight
                int errorReducedX = width/2 + (prevMidPoint.x - width/2) * straightDecay;
                midPoint = cv::Point(errorReducedX, height * 2 / 3);
            }
        } else {
            // No previous point - use center
            midPoint = cv::Point(width / 2, height * 2 / 3);
        }
    }

    // Update midPoint for next frame
    prevMidPoint = midPoint;
    prevMidCurve = midCurve;
    firstFrame = false;

    // Calculate normalized lateral error (-1.0 to 1.0) with improved dampening
    float centerX = width / 2;
    float rawError = (midPoint.x - centerX) / (width / 2.0f);
    
    // Apply rate limiting to error changes
    static float prevError = 0.0f;
    const float MAX_ERROR_CHANGE = 0.15f; // Maximum allowed change per frame
    
    float errorChange = rawError - prevError;
    if (std::abs(errorChange) > MAX_ERROR_CHANGE) {
        errorChange = (errorChange > 0) ? MAX_ERROR_CHANGE : -MAX_ERROR_CHANGE;
    }
    
    float lateralError = prevError + errorChange;
    prevError = lateralError;
    
    // Publish error to control system if needed
    if (publisher_) {
        publisher_->publishCameraError(lateralError);
        //publisher_->publishLanes(leftCurve, rightCurve);
    }

    // Draw the final lane visualization
    drawLanes(frame, leftCurve, rightCurve);
    
    // Draw center lane and reference point
    if (!midCurve.empty()) {
        for (size_t i = 1; i < midCurve.size(); i++) {
            cv::line(frame, midCurve[i-1], midCurve[i], cv::Scalar(0, 0, 255), 2);
        }
    }
    
    // Draw the reference point
    cv::circle(frame, midPoint, 8, cv::Scalar(255, 0, 255), -1);
    
    // Display lateral error as text
    std::string errorText = "Error: " + std::to_string(lateralError);
    cv::putText(frame, errorText, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 
                0.7, cv::Scalar(255, 255, 255), 2);
}


void LaneDetector::drawLanes(cv::Mat& frame, 
                            const std::vector<cv::Point>& leftCurve, 
                            const std::vector<cv::Point>& rightCurve)
{

    // Draw original points (that were used to calculate the lanes)
    // for (const auto& pt : prevLeftPoints) {
    //     cv::circle(frame, pt, 3, cv::Scalar(125, 0, 0), -1); // Dark blue for left points
    // }
    
    // for (const auto& pt : prevRightPoints) {
    //     cv::circle(frame, pt, 3, cv::Scalar(0, 125, 0), -1); // Dark green for right points
    // }
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

// void LaneDetector::clusterLanePoints(const std::vector<cv::Point>& points, 
//                                      std::vector<cv::Point>& leftPoints,
//                                      std::vector<cv::Point>& rightPoints,
//                                      cv::Mat& frame)
// {
//     if (points.empty()) {
//         return;
//     }

//     // Clear output vectors
//     leftPoints.clear();
//     rightPoints.clear();
    
//     int midX = frame.cols / 2;
//     int height = frame.rows;
    
//     // Pre-filtering - focus on lower part of the image
//     std::vector<cv::Point> filteredPoints;
//     for (const auto& pt : points) {
//         if (pt.y > height * 0.25) { // Keep points in lower 75% of image
//             filteredPoints.push_back(pt);
//         }    
//     }
    
//     if (filteredPoints.size() < 10) {
//         if (!prevLeftPoints.empty() && !prevRightPoints.empty()) {
//             leftPoints = prevLeftPoints;
//             rightPoints = prevRightPoints;
//         }
//         return;
//     }
    
//     // 2. CURVE-ORIENTED HISTORY PROJECTION
//     bool useHistory = !prevLeftCurve.empty() && !prevRightCurve.empty();
//     std::vector<cv::Point> projectedLeftLane, projectedRightLane;
    
//     if (useHistory) {
//         // Create projection of previous lanes with denser sampling
//         for (int y = height; y >= height * 0.33; y -= 10) { // Changed from 20 to 10 for denser points
//             int leftX = -1, rightX = -1;
            
//             for (const auto& pt : prevLeftCurve) {
//                 if (abs(pt.y - y) < 10) { // Reduced from 20 to 10
//                     leftX = pt.x;
//                     break;
//                 }
//             }
            
//             for (const auto& pt : prevRightCurve) {
//                 if (abs(pt.y - y) < 10) { // Reduced from 20 to 10
//                     rightX = pt.x;
//                     break;
//                 }
//             }
            
//             if (leftX != -1) {
//                 projectedLeftLane.push_back(cv::Point(leftX, y));
//             }
            
//             if (rightX != -1) {
//                 projectedRightLane.push_back(cv::Point(rightX, y));
//             }
//         }
//     }
    
//     // Sort points by y-coordinate (bottom to top)
//     std::sort(filteredPoints.begin(), filteredPoints.end(), 
//              [](const cv::Point& a, const cv::Point& b) { return a.y > b.y; });

//     // First pass: Check if all points might be the same lane based on proximity
//     bool allPointsPotentiallySameLane = false;

//     if (filteredPoints.size() >= 10) {
//         // Calculate average distance between points
//         double totalDistance = 0;
//         int numPairs = 0;
        
//         // Sample pairs of points
//         for (size_t i = 0; i < filteredPoints.size(); i += 2) {
//             for (size_t j = i + 1; j < std::min(i + 5, filteredPoints.size()); j++) {
//                 totalDistance += std::sqrt(std::pow(filteredPoints[i].x - filteredPoints[j].x, 2) + 
//                                         std::pow(filteredPoints[i].y - filteredPoints[j].y, 2));
//                 numPairs++;
//             }
//         }
        
//         // If average distance is small, points are likely from same lane
//         if (numPairs > 0) {
//             double avgDistance = totalDistance / numPairs;
//             if (avgDistance < frame.cols * 0.15) { // Points are close to each other
//                 allPointsPotentiallySameLane = true;
//             }
//         }
//     }

//     // If points appear to be from the same lane, identify which lane
//     if (allPointsPotentiallySameLane) {
//         // Calculate distances to previous left and right lanes
//         double avgDistToLeft = 0, avgDistToRight = 0;
//         int leftCount = 0, rightCount = 0;
        
//         for (const auto& pt : filteredPoints) {
//             double minDistLeft = std::numeric_limits<double>::max();
//             double minDistRight = std::numeric_limits<double>::max();
            
//             if (!prevLeftCurve.empty()) {
//                 for (const auto& prevPt : prevLeftCurve) {
//                     double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) * 0.6 + 
//                                         std::pow(pt.y - prevPt.y, 2) * 0.4);
//                     minDistLeft = std::min(minDistLeft, dist);
//                 }
//                 avgDistToLeft += minDistLeft;
//                 leftCount++;
//             }
            
//             if (!prevRightCurve.empty()) {
//                 for (const auto& prevPt : prevRightCurve) {
//                     double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) * 0.6 + 
//                                         std::pow(pt.y - prevPt.y, 2) * 0.4);
//                     minDistRight = std::min(minDistRight, dist);
//                 }
//                 avgDistToRight += minDistRight;
//                 rightCount++;
//             }
//         }
        
//         // Determine which lane they likely belong to
//         if (leftCount > 0) avgDistToLeft /= leftCount;
//         if (rightCount > 0) avgDistToRight /= rightCount;
        
//         if (avgDistToLeft < avgDistToRight && avgDistToLeft < frame.cols * 0.15) {
//             // All points are likely from left lane
//             leftPoints = filteredPoints;
//             rightPoints.clear();
//         } 
//         else if (avgDistToRight < avgDistToLeft && avgDistToRight < frame.cols * 0.15) {
//             // All points are likely from right lane
//             rightPoints = filteredPoints;
//             leftPoints.clear();
//         }
//     }

//     if (useHistory && !projectedLeftLane.empty() && !projectedRightLane.empty()) {
//         // First pass: just group the points using distance ratio like before
//         std::vector<cv::Point> potentialLeftPoints, potentialRightPoints;
        
//         for (const auto& pt : filteredPoints) {
//             double minDistLeft = std::numeric_limits<double>::max();
//             double minDistRight = std::numeric_limits<double>::max();
            
//             for (const auto& projPt : projectedLeftLane) {
//                 double dist = std::sqrt(std::pow(pt.x - projPt.x, 2) * 0.8 + 
//                                        std::pow(pt.y - projPt.y, 2) * 0.2);
//                 minDistLeft = std::min(minDistLeft, dist);
//             }
            
//             for (const auto& projPt : projectedRightLane) {
//                 double dist = std::sqrt(std::pow(pt.x - projPt.x, 2) * 0.8 + 
//                                        std::pow(pt.y - projPt.y, 2) * 0.2);
//                 minDistRight = std::min(minDistRight, dist);
//             }
            
//             bool assignedToGroup = false;

//             if (!potentialLeftPoints.empty()) {
//                 for (const auto& leftPt : potentialLeftPoints) {
//                     double pointDist = std::sqrt(std::pow(pt.x - leftPt.x, 2) + 
//                                                std::pow(pt.y - leftPt.y, 2));
                    
//                     // If very close to an existing left point, assign to left
//                     if (pointDist < frame.cols * 0.06) { // 6% of frame width
//                         potentialLeftPoints.push_back(pt);
//                         assignedToGroup = true;
//                         break;
//                     }
//                 }
//             }
            
//             if (!assignedToGroup && !potentialRightPoints.empty()) {
//                 for (const auto& rightPt : potentialRightPoints) {
//                     double pointDist = std::sqrt(std::pow(pt.x - rightPt.x, 2) + 
//                                                std::pow(pt.y - rightPt.y, 2));
                    
//                     // If very close to an existing right point, assign to right
//                     if (pointDist < frame.cols * 0.06) { // 6% of frame width
//                         potentialRightPoints.push_back(pt);
//                         assignedToGroup = true;
//                         break;
//                     }
//                 }
//             }
            
//             // Only if not already assigned based on proximity to other points
//             if (!assignedToGroup) {
//                 // Original distance ratio assignment
//                 double distanceRatio = minDistLeft / (minDistLeft + minDistRight);
                
//                 if (distanceRatio < 0.5) {
//                     potentialLeftPoints.push_back(pt);
//                 } else {
//                     potentialRightPoints.push_back(pt);
//                 }
//             }
//         }
        
//         // If both sides have some points, analyze distribution
//         if (potentialLeftPoints.size() > 3 && potentialRightPoints.size() > 3) {
//             leftPoints = potentialLeftPoints;
//             rightPoints = potentialRightPoints;
//         }
//         // Handle case where too many points went to one side
//         // Handle case where too many points went to one side
//         else if (potentialLeftPoints.size() <= 3 && potentialRightPoints.size() > 10) {
//             // Check if these are truly right lane points by looking at lane continuity
//             bool probablyAllRightLane = true;
            
//             if (!prevRightCurve.empty() && prevRightCurve.size() >= 3) {
//                 // Track how many points are close to the previous right lane
//                 int pointsNearRightLane = 0;
//                 int totalPoints = potentialRightPoints.size();
                
//                 for (const auto& pt : potentialRightPoints) {
//                     double minDist = std::numeric_limits<double>::max();
//                     for (const auto& prevPt : prevRightCurve) {
//                         double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) * 0.7 + 
//                                             std::pow(pt.y - prevPt.y, 2) * 0.3);
//                         minDist = std::min(minDist, dist);
//                     }
                    
//                     // Count points that are within a reasonable distance
//                     if (minDist < frame.cols * 0.2) {
//                         pointsNearRightLane++;
//                     }
//                 }
                
//                 // If a significant percentage of points are near the right lane, they're all right lane
//                 double percentNearRightLane = (double)pointsNearRightLane / totalPoints;
//                 if (percentNearRightLane > 0.7) { // If 70% or more points are near the right lane
//                     rightPoints = potentialRightPoints;
//                     // Left lane is actually missing - don't force assignment
//                     leftPoints.clear();
//                     probablyAllRightLane = true;
//                 } else {
//                     probablyAllRightLane = false;
//                 }
//             }
//             (void) probablyAllRightLane;
//             // // Only use the forced split if we're not confident they're all right lane
//             // if (!probablyAllRightLane) {
//             //     std::sort(filteredPoints.begin(), filteredPoints.end(),
//             //             [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
                
//             //     // Take the leftmost 40% of points for left lane
//             //     int leftCount = filteredPoints.size() * 0.4;
//             //     if (leftCount < 3) leftCount = 3;
                
//             //     leftPoints.assign(filteredPoints.begin(), filteredPoints.begin() + leftCount);
//             //     rightPoints.assign(filteredPoints.begin() + leftCount, filteredPoints.end());
//             // }
//         }
//         // Similarly for the other case
//         else if (potentialRightPoints.size() <= 3 && potentialLeftPoints.size() > 10) {
//             bool probablyAllLeftLane = true;
            
//             if (!prevLeftCurve.empty() && prevLeftCurve.size() >= 3) {
//                 int pointsNearLeftLane = 0;
//                 int totalPoints = potentialLeftPoints.size();
                
//                 for (const auto& pt : potentialLeftPoints) {
//                     double minDist = std::numeric_limits<double>::max();
//                     for (const auto& prevPt : prevLeftCurve) {
//                         double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) * 0.7 + 
//                                             std::pow(pt.y - prevPt.y, 2) * 0.3);
//                         minDist = std::min(minDist, dist);
//                     }
                    
//                     if (minDist < frame.cols * 0.2) { // More tolerant threshold
//                         pointsNearLeftLane++;
//                     }
//                 }
                
//                 double percentNearLeftLane = (double)pointsNearLeftLane / totalPoints;
//                 if (percentNearLeftLane > 0.7) { // 70% threshold
//                     leftPoints = potentialLeftPoints;
//                     rightPoints.clear();
//                     probablyAllLeftLane = true;
//                 } else {
//                     probablyAllLeftLane = false;
//                 }
//             }
//             (void) probablyAllLeftLane;
//             // // Only use the forced split if we're not confident they're all left lane
//             // if (!probablyAllLeftLane) {
//             //     std::sort(filteredPoints.begin(), filteredPoints.end(),
//             //             [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
                
//             //     // Take the rightmost 40% of points for right lane
//             //     int rightCount = filteredPoints.size() * 0.4;
//             //     if (rightCount < 3) rightCount = 3;
                
//             //     leftPoints.assign(filteredPoints.begin(), 
//             //                     filteredPoints.end() - rightCount);
//             //     rightPoints.assign(filteredPoints.end() - rightCount, 
//             //                     filteredPoints.end());
//             // }
//         }
//         else {
//             // Fall back to position-based clustering
//             for (const auto& pt : filteredPoints) {
//                 if (pt.x < midX) {
//                     leftPoints.push_back(pt);
//                 } else {
//                     rightPoints.push_back(pt);
//                 }
//             }
//         }
//     } 
//     else {
//         // No history - use simple left/right division with buffer
//         for (const auto& pt : filteredPoints) {
//             if (pt.x < midX - midX * 0.1) {  // Left 40% of screen
//                 leftPoints.push_back(pt);
//             } 
//             else if (pt.x > midX + midX * 0.1) {  // Right 40% of screen
//                 rightPoints.push_back(pt);
//             }
//             // Middle 20% is ignored as it's often ambiguous
//         }
//     }
    
//     // Sanity check: ensure lanes don't cross
//     if (leftPoints.size() >= 3 && rightPoints.size() >= 3) {
//         double leftMeanX = 0, rightMeanX = 0;
        
//         for (const auto& pt : leftPoints) {
//             leftMeanX += pt.x;
//         }
//         leftMeanX /= leftPoints.size();
        
//         for (const auto& pt : rightPoints) {
//             rightMeanX += pt.x;
//         }
//         rightMeanX /= rightPoints.size();
        
//         // If left is to the right of right, something is wrong
//         if (leftMeanX > rightMeanX) {
//             // In extreme curves, we might need to start over with simple position
//             leftPoints.clear();
//             rightPoints.clear();
            
//             for (const auto& pt : filteredPoints) {
//                 if (pt.x < midX) {
//                     leftPoints.push_back(pt);
//                 } else {
//                     rightPoints.push_back(pt);
//                 }
//             }
//         }
//     }
    
//     // Fallback to previous points if needed
//     if (leftPoints.size() < 3 && !prevLeftPoints.empty()) {
//         leftPoints = prevLeftPoints;
//     }
    
//     if (rightPoints.size() < 3 && !prevRightPoints.empty()) {
//         rightPoints = prevRightPoints;
//     }
    
//     // Save for next frame
//     if (leftPoints.size() >= 3) prevLeftPoints = leftPoints;
//     if (rightPoints.size() >= 3) prevRightPoints = rightPoints;
// }

void LaneDetector::clusterLanePoints(const std::vector<cv::Point>& points, 
                                     std::vector<cv::Point>& leftPoints,
                                     std::vector<cv::Point>& rightPoints,
                                     cv::Mat& frame)
{
    if (points.empty()) {
        return;
    }

    // Clear output vectors
    leftPoints.clear();
    rightPoints.clear();
    
    int midX = frame.cols / 2;
    int height = frame.rows;
    
    // Pre-filtering - focus on lower part of the image
    std::vector<cv::Point> filteredPoints;
    for (const auto& pt : points) {
        if (pt.y > height * 0.25) { // Keep points in lower 75% of image
            filteredPoints.push_back(pt);
        }    
    }
    
    if (filteredPoints.size() < 10) {
        if (!prevLeftPoints.empty() && !prevRightPoints.empty()) {
            leftPoints = prevLeftPoints;
            rightPoints = prevRightPoints;
        }
        return;
    }
    
    // 1. HISTORY PROJECTION USING POLYNOMIAL FITTING
    bool useHistory = !prevLeftCurve.empty() && !prevRightCurve.empty();
    std::vector<cv::Point> projectedLeftLane, projectedRightLane;
    
    if (useHistory) {
        // Prepare vectors for polynomial fitting for left lane
        std::vector<double> leftYs, leftXs;
        for (const auto& pt : prevLeftCurve) {
            leftYs.push_back(pt.y);
            leftXs.push_back(pt.x);
        }
        // Convert vectors to cv::Mat (as column matrices)
        cv::Mat leftYMat(leftYs), leftXMat(leftXs);
        cv::Mat coeffsLeft = polyfit(leftYMat, leftXMat, 2);
        
        if (!coeffsLeft.empty() && coeffsLeft.rows >= 3) {
            for (int y = height; y >= int(height * 0.33); y -= 10) {
                double x = coeffsLeft.at<double>(0) * y * y +
                           coeffsLeft.at<double>(1) * y +
                           coeffsLeft.at<double>(2);
                projectedLeftLane.push_back(cv::Point(round(x), y));
            }
        }
        
        // Do the same for right lane
        std::vector<double> rightYs, rightXs;
        for (const auto& pt : prevRightCurve) {
            rightYs.push_back(pt.y);
            rightXs.push_back(pt.x);
        }
        cv::Mat rightYMat(rightYs), rightXMat(rightXs);
        cv::Mat coeffsRight = polyfit(rightYMat, rightXMat, 2);
        
        if (!coeffsRight.empty() && coeffsRight.rows >= 3) {
            for (int y = height; y >= int(height * 0.33); y -= 10) {
                double x = coeffsRight.at<double>(0) * y * y +
                           coeffsRight.at<double>(1) * y +
                           coeffsRight.at<double>(2);
                projectedRightLane.push_back(cv::Point(round(x), y));
            }
        }
    }
    
    // Compute adaptive midline based on history projection if available
    int adaptiveMidX = midX;
    if (useHistory && !projectedLeftLane.empty() && !projectedRightLane.empty()) {
        // Use the bottom-most (largest y) points of the projected curves
        int leftXProj = projectedLeftLane.front().x;
        int rightXProj = projectedRightLane.front().x;
        adaptiveMidX = (leftXProj + rightXProj) / 2;
    }
    
    // Sort filtered points by y-coordinate (bottom to top)
    std::sort(filteredPoints.begin(), filteredPoints.end(), 
             [](const cv::Point& a, const cv::Point& b) { return a.y > b.y; });

    // First pass: Check if all points might be from the same lane based on proximity
    bool allPointsPotentiallySameLane = false;
    if (filteredPoints.size() >= 10) {
        double totalDistance = 0;
        int numPairs = 0;
        for (size_t i = 0; i < filteredPoints.size(); i += 2) {
            for (size_t j = i + 1; j < std::min(i + 5, filteredPoints.size()); j++) {
                totalDistance += std::sqrt(std::pow(filteredPoints[i].x - filteredPoints[j].x, 2) + 
                                           std::pow(filteredPoints[i].y - filteredPoints[j].y, 2));
                numPairs++;
            }
        }
        if (numPairs > 0) {
            double avgDistance = totalDistance / numPairs;
            if (avgDistance < frame.cols * 0.15) {
                allPointsPotentiallySameLane = true;
            }
        }
    }

    // If points seem to come from one lane, decide based on distances to previous curves
    if (allPointsPotentiallySameLane) {
        double avgDistToLeft = 0, avgDistToRight = 0;
        int leftCount = 0, rightCount = 0;
        
        for (const auto& pt : filteredPoints) {
            double minDistLeft = std::numeric_limits<double>::max();
            double minDistRight = std::numeric_limits<double>::max();
            
            if (!prevLeftCurve.empty()) {
                for (const auto& prevPt : prevLeftCurve) {
                    double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) * 0.6 + 
                                            std::pow(pt.y - prevPt.y, 2) * 0.4);
                    minDistLeft = std::min(minDistLeft, dist);
                }
                avgDistToLeft += minDistLeft;
                leftCount++;
            }
            
            if (!prevRightCurve.empty()) {
                for (const auto& prevPt : prevRightCurve) {
                    double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) * 0.6 + 
                                            std::pow(pt.y - prevPt.y, 2) * 0.4);
                    minDistRight = std::min(minDistRight, dist);
                }
                avgDistToRight += minDistRight;
                rightCount++;
            }
        }
        
        if (leftCount > 0) avgDistToLeft /= leftCount;
        if (rightCount > 0) avgDistToRight /= rightCount;
        
        if (avgDistToLeft < avgDistToRight && avgDistToLeft < frame.cols * 0.15) {
            leftPoints = filteredPoints;
            rightPoints.clear();
        } else if (avgDistToRight < avgDistToLeft && avgDistToRight < frame.cols * 0.15) {
            rightPoints = filteredPoints;
            leftPoints.clear();
        }
    }

    // 2. POINT GROUPING USING HISTORY PROJECTION
    if (useHistory && !projectedLeftLane.empty() && !projectedRightLane.empty()) {
        std::vector<cv::Point> potentialLeftPoints, potentialRightPoints;
        
        for (const auto& pt : filteredPoints) {
            double minDistLeft = std::numeric_limits<double>::max();
            double minDistRight = std::numeric_limits<double>::max();
            
            for (const auto& projPt : projectedLeftLane) {
                double dist = std::sqrt(std::pow(pt.x - projPt.x, 2) * 0.8 +
                                        std::pow(pt.y - projPt.y, 2) * 0.2);
                minDistLeft = std::min(minDistLeft, dist);
            }
            
            for (const auto& projPt : projectedRightLane) {
                double dist = std::sqrt(std::pow(pt.x - projPt.x, 2) * 0.8 +
                                        std::pow(pt.y - projPt.y, 2) * 0.2);
                minDistRight = std::min(minDistRight, dist);
            }
            
            bool assignedToGroup = false;
            if (!potentialLeftPoints.empty()) {
                for (const auto& leftPt : potentialLeftPoints) {
                    double pointDist = std::sqrt(std::pow(pt.x - leftPt.x, 2) +
                                                 std::pow(pt.y - leftPt.y, 2));
                    if (pointDist < frame.cols * 0.06) {
                        potentialLeftPoints.push_back(pt);
                        assignedToGroup = true;
                        break;
                    }
                }
            }
            if (!assignedToGroup && !potentialRightPoints.empty()) {
                for (const auto& rightPt : potentialRightPoints) {
                    double pointDist = std::sqrt(std::pow(pt.x - rightPt.x, 2) +
                                                 std::pow(pt.y - rightPt.y, 2));
                    if (pointDist < frame.cols * 0.06) {
                        potentialRightPoints.push_back(pt);
                        assignedToGroup = true;
                        break;
                    }
                }
            }
            if (!assignedToGroup) {
                double distanceRatio = minDistLeft / (minDistLeft + minDistRight);
                if (distanceRatio < 0.5) {
                    potentialLeftPoints.push_back(pt);
                } else {
                    potentialRightPoints.push_back(pt);
                }
            }
        }
        
        if (potentialLeftPoints.size() > 3 && potentialRightPoints.size() > 3) {
            leftPoints = potentialLeftPoints;
            rightPoints = potentialRightPoints;
        }
        else if (potentialLeftPoints.size() <= 3 && potentialRightPoints.size() > 10) {
            bool probablyAllRightLane = true;
            if (!prevRightCurve.empty() && prevRightCurve.size() >= 3) {
                int pointsNearRightLane = 0;
                int totalPoints = potentialRightPoints.size();
                for (const auto& pt : potentialRightPoints) {
                    double minDist = std::numeric_limits<double>::max();
                    for (const auto& prevPt : prevRightCurve) {
                        double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) * 0.7 +
                                                std::pow(pt.y - prevPt.y, 2) * 0.3);
                        minDist = std::min(minDist, dist);
                    }
                    if (minDist < frame.cols * 0.2) {
                        pointsNearRightLane++;
                    }
                }
                double percentNearRightLane = (double)pointsNearRightLane / totalPoints;
                if (percentNearRightLane > 0.7) {
                    rightPoints = potentialRightPoints;
                    leftPoints.clear();
                    probablyAllRightLane = true;
                } else {
                    probablyAllRightLane = false;
                }
            }
            (void) probablyAllRightLane;
        }
        else if (potentialRightPoints.size() <= 3 && potentialLeftPoints.size() > 10) {
            bool probablyAllLeftLane = true;
            if (!prevLeftCurve.empty() && prevLeftCurve.size() >= 3) {
                int pointsNearLeftLane = 0;
                int totalPoints = potentialLeftPoints.size();
                for (const auto& pt : potentialLeftPoints) {
                    double minDist = std::numeric_limits<double>::max();
                    for (const auto& prevPt : prevLeftCurve) {
                        double dist = std::sqrt(std::pow(pt.x - prevPt.x, 2) * 0.7 +
                                                std::pow(pt.y - prevPt.y, 2) * 0.3);
                        minDist = std::min(minDist, dist);
                    }
                    if (minDist < frame.cols * 0.2) {
                        pointsNearLeftLane++;
                    }
                }
                double percentNearLeftLane = (double)pointsNearLeftLane / totalPoints;
                if (percentNearLeftLane > 0.7) {
                    leftPoints = potentialLeftPoints;
                    rightPoints.clear();
                    probablyAllLeftLane = true;
                } else {
                    probablyAllLeftLane = false;
                }
            }
            (void) probablyAllLeftLane;
        }
        else {
            // Fallback: position-based clustering using fixed midline
            for (const auto& pt : filteredPoints) {
                if (pt.x < midX) {
                    leftPoints.push_back(pt);
                } else {
                    rightPoints.push_back(pt);
                }
            }
        }
    } 
    else {
        // No history - use simple left/right division using adaptive midline
        for (const auto& pt : filteredPoints) {
            if (pt.x < adaptiveMidX - adaptiveMidX * 0.1) {  // left 40% relative to adaptiveMidX
                leftPoints.push_back(pt);
            } 
            else if (pt.x > adaptiveMidX + adaptiveMidX * 0.1) {  // right 40% relative to adaptiveMidX
                rightPoints.push_back(pt);
            }
            // Middle 20% remains ambiguous and is ignored
        }
    }
    
    // Sanity check: ensure lanes don't cross
    if (leftPoints.size() >= 3 && rightPoints.size() >= 3) {
        double leftMeanX = 0, rightMeanX = 0;
        for (const auto& pt : leftPoints) {
            leftMeanX += pt.x;
        }
        leftMeanX /= leftPoints.size();
        for (const auto& pt : rightPoints) {
            rightMeanX += pt.x;
        }
        rightMeanX /= rightPoints.size();
        if (leftMeanX > rightMeanX) {
            leftPoints.clear();
            rightPoints.clear();
            for (const auto& pt : filteredPoints) {
                if (pt.x < midX) {
                    leftPoints.push_back(pt);
                } else {
                    rightPoints.push_back(pt);
                }
            }
        }
    }
    
    // Fallback to previous points if needed
    if (leftPoints.size() < 3 && !prevLeftPoints.empty()) {
        leftPoints = prevLeftPoints;
    }
    if (rightPoints.size() < 3 && !prevRightPoints.empty()) {
        rightPoints = prevRightPoints;
    }
    
    // Save for next frame
    if (leftPoints.size() >= 3) prevLeftPoints = leftPoints;
    if (rightPoints.size() >= 3) prevRightPoints = rightPoints;
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
        for (int y = frame.rows; y >= frame.rows * 0.35; y -= 5) {
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
    cv::setIdentity(leftLaneKF.processNoiseCov, cv::Scalar(1e-4));
    cv::setIdentity(rightLaneKF.processNoiseCov, cv::Scalar(1e-4));
    
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
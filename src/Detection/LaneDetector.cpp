#include "LaneDetector.hpp"
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

LaneDetector::LaneDetector(const std::string& enginePath, std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));

    createExecutionContext(enginePath);

    // Set highest stream priority
    int leastPriority, greatestPriority;
    cudaDeviceGetStreamPriorityRange(&leastPriority, &greatestPriority);
    cudaStreamCreateWithPriority(&stream, cudaStreamNonBlocking,
                                 greatestPriority);

    // Create an OpenCV CUDA stream (with default flags)
    cv_stream = cv::cuda::Stream();

    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // Pin memory for faster transfers
    void* input_ptr;
    void* output_ptr;
    cudaHostAlloc(&input_ptr, 3 * HEIGHT * WIDTH * sizeof(float),
                  cudaHostAllocMapped);
    cudaHostAlloc(&output_ptr, 1 * HEIGHT * WIDTH * sizeof(float),
                  cudaHostAllocMapped);
    inputData  = static_cast<float*>(input_ptr);
    outputData = static_cast<float*>(output_ptr);

    // Allocate GPU memory
    size_t pitch;
    cudaMallocPitch(&inputDevice, &pitch, WIDTH * sizeof(float),
                    HEIGHT * 3);
    cudaMallocPitch(&outputDevice, &pitch, WIDTH * sizeof(float),
                    HEIGHT * 1);

    // Calibrate IPM

    float cameraHeight = 0.137f;       // meters
    float cameraPitch = 20.0f;       // degrees down from horizontal
    float horizontalFOV = 100.0f;     // degrees
    float img_height = static_cast<float>(HEIGHT);
    float img_width = static_cast<float>(WIDTH);
    float h_fov_rad = horizontalFOV * CV_PI / 180.0f;
    float verticalFOV = 2.0f * std::atan((img_height/img_width) * std::tan(h_fov_rad/2.0f)) * 180.0f / CV_PI;
    float nearDistance = 0.05f;       // meters
    float farDistance = 0.4f;       // meters
    float laneWidth = 1.5f;      // meters
    bevSize = cv::Size(WIDTH, HEIGHT);
    cv::Size origSize = cv::Size(WIDTH, HEIGHT);
    ipm.initialize(origSize, bevSize);
    ipm.calibrateFromCamera(cameraHeight, cameraPitch, horizontalFOV, verticalFOV,
                            nearDistance, farDistance, laneWidth);

    publisher_ = std::make_shared<LaneDetectorPublisher>(session_);

    try     
    {
        this->canBus     = new CAN();
        this->canBus->init("/dev/spidev0.0");
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing CAN on Object Detector " << e.what() << std::endl;
    }

}

LaneDetector::~LaneDetector()
{
    cudaFreeHost(inputData);
    cudaFreeHost(outputData);
    cudaFree(inputDevice);
    cudaFree(outputDevice);
    cudaStreamDestroy(stream);
}

void LaneDetector::createExecutionContext(const std::string& enginePath)
{
    Logger logger;

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

void LaneDetector::detect(cv::Mat& frame)
{
    float milliseconds = 0;

    // Preprocess
    preProcess(frame);

    cudaEventRecord(start, stream);

    // Copy to GPU
    cudaMemcpyAsync(inputDevice, inputData,
                    3 * HEIGHT * WIDTH * sizeof(float),
                    cudaMemcpyHostToDevice, stream);

    // Run inference with optimization flags
    void* bindings[] = {inputDevice, outputDevice};
    context->enqueueV2(bindings, stream, nullptr);

    // Copy back to CPU
    cudaMemcpyAsync(outputData, outputDevice,
                    1 * HEIGHT * WIDTH * sizeof(float),
                    cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);
    
    // Postprocess
    postProcess(frame);

    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Inference time: " << milliseconds << "ms\n";
}

void LaneDetector::preProcess(const cv::Mat& frame)
{
    // Create static GPU matrices
    static cv::cuda::GpuMat d_frame, d_resized, d_rgb_image;
    static cv::Mat cpu_rgb_image(HEIGHT, WIDTH, CV_8UC3);

    // Upload input frame to GPU
    d_frame.upload(frame);

    // Resize on GPU with CUDA stream
    cv::cuda::resize(d_frame, d_resized, cv::Size(WIDTH, HEIGHT), 0, 0,
                     cv::INTER_NEAREST, cv_stream);

    // Convert BGR to RGB on GPU
    cv::cuda::cvtColor(d_resized, d_rgb_image, cv::COLOR_BGR2RGB, 0, cv_stream);

    // Download the result back to CPU
    d_rgb_image.download(cpu_rgb_image, cv_stream);

    // Wait for CUDA operations to complete
    cv_stream.waitForCompletion();

    // Continue with existing channel reordering code
    const int plane_size      = HEIGHT * WIDTH;
    const uint8_t* frame_data = cpu_rgb_image.data;

    for (int c = 0; c < 3; c++)
    {
        for (int i = 0; i < plane_size; i++)
        {
            inputData[c * plane_size + i] = frame_data[i * 3 + c] / 255.0f;
        }
    }
}

void LaneDetector::postProcess(cv::Mat& frame)
{
    static cv::Mat binary_mask(HEIGHT, WIDTH, CV_8UC1);
    const int total_pixels = HEIGHT * WIDTH;

    for (int i = 0; i < total_pixels; i++) {
        int y = i / WIDTH;
        int x = i % WIDTH;
        uchar value = (outputData[i] > 0.5) ? 127 : 0;
        binary_mask.at<uchar>(y, x) = value;
    }

    // Apply IPM to mask and frame
    cv::Mat ipm_mask = ipm.applyIPM(binary_mask);
    cv::Mat ipm_frame = ipm.applyIPM(frame);
    
    cv::Mat resized_ipm_mask;
    cv::resize(ipm_mask, resized_ipm_mask, frame.size(), 0, 0, cv::INTER_NEAREST);

    cv::Mat resized_ipm_frame;
    cv::resize(ipm_frame, resized_ipm_frame, frame.size(), 0, 0, cv::INTER_NEAREST);

    resized_ipm_frame.copyTo(frame);

    createLanes(resized_ipm_mask, frame);
}

void LaneDetector::createLanes(cv::Mat& binary_mask, cv::Mat& frame)
{
    std::vector<std::vector<cv::Point>> lanePolylines = clusterLaneMask(binary_mask, 30, 40, 6);
    
    float maxHorizontalDistance = frame.cols * 0.15; // 15% of frame width
    float maxVerticalGap = frame.rows * 0.35;       // 35% of frame height
    mergeLaneComponents(lanePolylines, maxHorizontalDistance, maxVerticalGap);

    if (lanePolylines.size() > 2) {
        // Sort by number of points (largest first)
        std::sort(lanePolylines.begin(), lanePolylines.end(), 
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return a.size() > b.size();
            });
        lanePolylines.resize(2);
    }

    cv::Mat allPolylinesViz = frame.clone();
    drawPolyLanes(lanePolylines, allPolylinesViz);

    currentFrame++;

    std::vector<cv::Point> leftCurve, rightCurve;
    
    if (lanePolylines.size() == 2) {
        // Find the lowest point (highest y-value) in each polyline
        cv::Point lowestPoint1(-1, -1);
        cv::Point lowestPoint2(-1, -1);
        
        // Find lowest point in first polyline
        for (const auto& pt : lanePolylines[0]) {
            if (pt.y > lowestPoint1.y) {
                lowestPoint1 = pt;
            }
        }
        
        // Find lowest point in second polyline
        for (const auto& pt : lanePolylines[1]) {
            if (pt.y > lowestPoint2.y) {
                lowestPoint2 = pt;
            }
        }
                
        // Debug visualization of lowest points
        cv::circle(allPolylinesViz, lowestPoint1, 8, cv::Scalar(255, 0, 255), -1);
        cv::circle(allPolylinesViz, lowestPoint2, 8, cv::Scalar(0, 255, 255), -1);
        
        // Compare x-coordinates to determine left/right
        if (lowestPoint1.x < lowestPoint2.x) {
            leftCurve = lanePolylines[0];
            rightCurve = lanePolylines[1];
            
            // Debug text
            std::string leftText = "Left: " + std::to_string(lowestPoint1.x);
            std::string rightText = "Right: " + std::to_string(lowestPoint2.x);
            cv::putText(allPolylinesViz, leftText, lowestPoint1 + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
            cv::putText(allPolylinesViz, rightText, lowestPoint2 + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        } else {
            leftCurve = lanePolylines[1];
            rightCurve = lanePolylines[0];
            
            // Debug text
            std::string leftText = "Left: " + std::to_string(lowestPoint2.x);
            std::string rightText = "Right: " + std::to_string(lowestPoint1.x);
            cv::putText(allPolylinesViz, leftText, lowestPoint2 + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
            cv::putText(allPolylinesViz, rightText, lowestPoint1 + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }

        if (!leftCurve.empty() && !rightCurve.empty()) {
            // Calculate actual lane width at several points
            float totalWidth = 0.0f;
            int measurementCount = 0;
            
            // Sample at different points along the lane height
            for (int i = 0; i < 5; i++) {
                int targetY = frame.rows - frame.rows/5 - i*(frame.rows/10);  // From bottom to middle
                
                // Find closest points to this Y in both curves
                int leftIdx = -1, rightIdx = -1;
                int leftMinDist = INT_MAX, rightMinDist = INT_MAX;
                
                for (size_t j = 0; j < leftCurve.size(); j++) {
                    int dist = std::abs(leftCurve[j].y - targetY);
                    if (dist < leftMinDist) {
                        leftMinDist = dist;
                        leftIdx = j;
                    }
                }
                
                for (size_t j = 0; j < rightCurve.size(); j++) {
                    int dist = std::abs(rightCurve[j].y - targetY);
                    if (dist < rightMinDist) {
                        rightMinDist = dist;
                        rightIdx = j;
                    }
                }
                
                // If valid points found in both curves
                if (leftIdx >= 0 && rightIdx >= 0) {
                    float width = rightCurve[rightIdx].x - leftCurve[leftIdx].x;
                    // Basic sanity check - lane should be positive width and reasonable size
                    if (width > MIN_VALID_WIDTH && width < frame.cols * 0.8f) {
                        totalWidth += width;
                        measurementCount++;
                    }
                }
            }
            
            // Update lane width with moving average
            if (measurementCount > 0) {
                float measuredWidth = totalWidth / measurementCount;
                
                // Add to history queue
                recentLaneWidths.push_back(measuredWidth);
                if (recentLaneWidths.size() > static_cast<uint8_t>(MAX_WIDTH_HISTORY)) {
                    recentLaneWidths.pop_front(); // Remove oldest
                }
                
                // Calculate average
                float sumWidth = 0.0f;
                for (const auto& w : recentLaneWidths) {
                    sumWidth += w;
                }
                avgLaneWidth = sumWidth / recentLaneWidths.size();
                
                // Display the calculated lane width for debugging
                std::string widthMsg = "Lane width: " + std::to_string(static_cast<int>(avgLaneWidth)) + "px";
                cv::putText(allPolylinesViz, widthMsg, cv::Point(20, 120), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
            }
        }

        // Limit the maximum curve drift from center
        if (!leftCurve.empty() && !rightCurve.empty()) {
            int width = frame.cols;
            float centerX = width / 2.0f;
            float maxOffsetDistance = width * 0.3f; // Maximum allowed offset (30% of frame width)
            
            // Calculate current lane midpoint at each y-level
            for (size_t i = 0; i < std::min(leftCurve.size(), rightCurve.size()); i++) {
                size_t leftIdx = i * leftCurve.size() / std::min(leftCurve.size(), rightCurve.size());
                size_t rightIdx = i * rightCurve.size() / std::min(leftCurve.size(), rightCurve.size());
                
                float midX = (leftCurve[leftIdx].x + rightCurve[rightIdx].x) / 2.0f;
                float offset = midX - centerX;
                
                // If offset exceeds limit, adjust both lane curves
                if (std::abs(offset) > maxOffsetDistance) {
                    float adjustment = offset - (offset > 0 ? maxOffsetDistance : -maxOffsetDistance);
                    
                    // Apply adjustment to this point in both curves
                    leftCurve[leftIdx].x -= adjustment;
                    rightCurve[rightIdx].x -= adjustment;
                }
            }
        }

        prevLeftCurve = leftCurve;
        prevRightCurve = rightCurve;
        
        leftLaneLastUpdatedFrame = currentFrame;
        rightLaneLastUpdatedFrame = currentFrame;
    } else if (lanePolylines.size() == 1) {
        cv::Point lowestPoint(-1, -1);
        int centerX = frame.cols / 2;
        float avgX = 0;
        
        // Find lowest point and average X position
        for (const auto& pt : lanePolylines[0]) {
            if (pt.y > lowestPoint.y) {
                lowestPoint = pt;
            }
            avgX += pt.x;
        }
        avgX /= lanePolylines[0].size();
        
        // Draw detected lane's lowest point
        cv::circle(allPolylinesViz, lowestPoint, 8, cv::Scalar(255, 0, 255), -1);
        
        float laneWidth = (avgLaneWidth > MIN_VALID_WIDTH) ? avgLaneWidth : frame.cols * 0.55f;
        
        // Determine if it's a left or right lane based on position
        bool isLeftLane = false;

        bool hasValidLeftMemory = (currentFrame - leftLaneLastUpdatedFrame) < MAX_LANE_MEMORY_FRAMES;
        bool hasValidRightMemory = (currentFrame - rightLaneLastUpdatedFrame) < MAX_LANE_MEMORY_FRAMES;
        
        // If we have previous lanes, use them to identify current lane
        if (hasValidLeftMemory || hasValidRightMemory) {
            
            float leftDistance = hasValidLeftMemory ? 
                        calculateLaneDistance(lanePolylines[0], prevLeftCurve) : 
                        FLT_MAX;
    
            float rightDistance = hasValidRightMemory ? 
                                calculateLaneDistance(lanePolylines[0], prevRightCurve) : 
                                FLT_MAX;

            if (hasValidLeftMemory) {
                float leftStaleness = 1.0f + 0.05f * (currentFrame - leftLaneLastUpdatedFrame);
                leftDistance *= leftStaleness;
            }
            
            if (hasValidRightMemory) {
                float rightStaleness = 1.0f + 0.05f * (currentFrame - rightLaneLastUpdatedFrame);
                rightDistance *= rightStaleness;
            }
            
            // Lower (adjusted) distance means better match
            isLeftLane = leftDistance < rightDistance;
            
            std::string debugMsg = "Memory match: " + std::string(isLeftLane ? "LEFT" : "RIGHT") + 
                                  " (L:" + std::to_string(leftDistance).substr(0,5) + 
                                  "/R:" + std::to_string(rightDistance).substr(0,5) + ")";
            cv::putText(allPolylinesViz, debugMsg, cv::Point(20, 80), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
            
            // Add staleness information
            std::string staleMsg = "Staleness - L:" + std::to_string(currentFrame - leftLaneLastUpdatedFrame) +
                                  " R:" + std::to_string(currentFrame - rightLaneLastUpdatedFrame);
            cv::putText(allPolylinesViz, staleMsg, cv::Point(20, 100), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        }
        else {
            // Fallback to position-based detection
            isLeftLane = avgX < centerX;
            
            // If memory is too old, add a notice
            if (!hasValidLeftMemory || !hasValidRightMemory) {
                cv::putText(allPolylinesViz, "Memory expired - using position", cv::Point(20, 80), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            } else {
                cv::putText(allPolylinesViz, "Position-based detection", cv::Point(20, 80), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
            }
        }
        
        if (isLeftLane) {
            leftCurve = lanePolylines[0];

            prevLeftCurve = leftCurve;
            leftLaneLastUpdatedFrame = currentFrame;
            cv::putText(allPolylinesViz, "Left Lane (Detected)", lowestPoint + cv::Point(10, 10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
            
            // Create synthetic right lane
            rightCurve.reserve(leftCurve.size());
            for (const auto& pt : leftCurve) {
                rightCurve.push_back(cv::Point(pt.x + laneWidth, pt.y));
            }
            
            // Visualize synthetic right lane
            for (size_t i = 1; i < rightCurve.size(); i++) {
                cv::line(allPolylinesViz, rightCurve[i-1], rightCurve[i], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
            cv::putText(allPolylinesViz, "Right Lane (Estimated)", rightCurve[rightCurve.size()/2] + cv::Point(10, 10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        } else {
            // The detected lane is the right lane
            rightCurve = lanePolylines[0];

            prevRightCurve = rightCurve;
            rightLaneLastUpdatedFrame = currentFrame;
            cv::putText(allPolylinesViz, "Right Lane (Detected)", lowestPoint + cv::Point(10, 10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            
            // Create synthetic left lane
            leftCurve.reserve(rightCurve.size());
            for (const auto& pt : rightCurve) {
                leftCurve.push_back(cv::Point(pt.x - laneWidth, pt.y));
            }
            
            // Visualize synthetic left lane
            for (size_t i = 1; i < leftCurve.size(); i++) {
                cv::line(allPolylinesViz, leftCurve[i-1], leftCurve[i], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
            }
            cv::putText(allPolylinesViz, "Left Lane (Estimated)", leftCurve[leftCurve.size()/2] + cv::Point(10, 10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
        }
        
        // Log that we're using a synthetic lane
        std::string statusMsg = isLeftLane ? "Using synthetic RIGHT lane" : "Using synthetic LEFT lane"; 
        cv::putText(allPolylinesViz, statusMsg, cv::Point(20, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    std::vector<cv::Point> midCurve;
    if (!leftCurve.empty() && !rightCurve.empty())
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
    }

    cv::Point midPoint;
    int height = frame.rows;
    int width  = frame.cols;

    if (!midCurve.empty()) {
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

        float centerX  = width / 2;
        float rawError = (midPoint.x - centerX) / (width / 2.0f);

        // Apply rate limiting to error changes
        static float prevError       = 0.0f;
        const float MAX_ERROR_CHANGE = 1.0f; // Maximum allowed change per frame

        float errorChange = rawError - prevError;
        if (std::abs(errorChange) > MAX_ERROR_CHANGE)
        {
            errorChange = (errorChange > 0) ? MAX_ERROR_CHANGE : -MAX_ERROR_CHANGE;
        }

        float lateralError = prevError + errorChange;
        prevError          = lateralError;

        const float MAX_ERROR = 3.0f;
        if (lateralError > MAX_ERROR) {
            lateralError = MAX_ERROR;
            prevError = MAX_ERROR; // Update prevError as well
        } else if (lateralError < -MAX_ERROR) {
            lateralError = -MAX_ERROR;
            prevError = -MAX_ERROR; // Update prevError as well
        }

        publisher_->publishCameraError(lateralError);

        std::string statusMsg = "Error: " + std::to_string(lateralError).substr(0, 6);
        cv::putText(allPolylinesViz, statusMsg, cv::Point(60, 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        // Draw middle lane curve with white color and thicker line
        cv::Scalar midCurveColor = cv::Scalar(255, 255, 255); // White
        for (size_t i = 1; i < midCurve.size(); i++) {
            cv::line(allPolylinesViz, midCurve[i-1], midCurve[i], midCurveColor, 3);
        }

        cv::circle(allPolylinesViz, midPoint, 8, cv::Scalar(255, 0, 255), -1);
    }

    allPolylinesViz.copyTo(frame);
}


std::vector<std::vector<cv::Point>> LaneDetector::clusterLaneMask(const cv::Mat& laneMask, int kernelSize, int minArea, int maxLanes) {    
    static cv::Mat verticalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize * 3));
    static cv::Mat horizontalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    
    cv::Mat result = laneMask.clone();
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, verticalKernel);
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, horizontalKernel);

    cv::Mat labels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(result, labels, stats, centroids, 8, CV_32S);
    
    std::vector<std::pair<int, float>> validComponents;
    validComponents.reserve(std::min(numLabels, maxLanes + 3));
    
    for (int i = 1; i < numLabels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > minArea) {
            float centerX = centroids.at<double>(i, 0);
            validComponents.push_back(std::make_pair(i, centerX));
        }
    }
    
    // Use partial sort instead of full sort when number of valid components bigger than maxLanes
    if (validComponents.size() > static_cast<size_t>(maxLanes)) {
        std::partial_sort(validComponents.begin(), validComponents.begin() + maxLanes, 
                        validComponents.end(), 
                        [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                            return a.second < b.second;
                        });
        validComponents.resize(maxLanes);
    } else {
        std::sort(validComponents.begin(), validComponents.end(), 
            [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second < b.second;
            });
    }
    
    // Reserve capacity for output
    std::vector<std::vector<cv::Point>> lanePolylines;
    lanePolylines.reserve(validComponents.size());
    
    // Process each lane with optimized extraction
    for (const auto& comp : validComponents) {
        int compIdx = comp.first;
        
        // Extract points more efficiently using row pointers
        std::vector<cv::Point> lanePoints;
        lanePoints.reserve(labels.rows/5);
        
        for (int y = 0; y < labels.rows; y += 2) {
            const int* row = labels.ptr<int>(y);
            int xStart = -1, xEnd = -1;
            
            for (int x = 0; x < labels.cols; x++) {
                if (row[x] == compIdx) {
                    if (xStart < 0) xStart = x;
                    xEnd = x;
                }
            }
            
            if (xStart >= 0) {
                int midX = (xStart + xEnd) / 2;
                lanePoints.push_back(cv::Point(midX, y));
            }
        }
        
        if (!lanePoints.empty()) {
            lanePolylines.push_back(std::move(lanePoints));
        }
    }
    
    return lanePolylines;
}

void LaneDetector::mergeLaneComponents(std::vector<std::vector<cv::Point>>& lanePolylines, float maxHorizontalDist, float maxVerticalGap) {
    if (lanePolylines.size() <= 1) return;
    
    bool mergePerformed = true;
    while (mergePerformed) {
        mergePerformed = false;
        
        for (size_t i = 0; i < lanePolylines.size() && !mergePerformed; i++) {
            for (size_t j = i + 1; j < lanePolylines.size() && !mergePerformed; j++) {
                // Compute the y-range and x-average of both polylines
                int minY1 = INT_MAX, maxY1 = 0;
                int minY2 = INT_MAX, maxY2 = 0;
                float avgX1 = 0, avgX2 = 0;
                
                for (const auto& pt : lanePolylines[i]) {
                    minY1 = std::min(minY1, pt.y);
                    maxY1 = std::max(maxY1, pt.y);
                    avgX1 += pt.x;
                }
                avgX1 /= lanePolylines[i].size();
                
                for (const auto& pt : lanePolylines[j]) {
                    minY2 = std::min(minY2, pt.y);
                    maxY2 = std::max(maxY2, pt.y);
                    avgX2 += pt.x;
                }
                avgX2 /= lanePolylines[j].size();
                
                // Check if they're horizontally aligned (similar X position)
                float hDist = std::abs(avgX1 - avgX2);
                
                // Calculate the vertical gap between segments
                float verticalGap;
                if (maxY1 < minY2) {
                    // First segment is above second segment
                    verticalGap = minY2 - maxY1;
                } else if (maxY2 < minY1) {
                    // Second segment is above first segment
                    verticalGap = minY1 - maxY2;
                } else {
                    // Segments overlap vertically - no gap
                    verticalGap = 0;
                }
                
                // Also check if they have similar directions (optional but recommended)
                bool similarDirection = true;
                if (!lanePolylines[i].empty() && lanePolylines[i].size() > 1 &&
                    !lanePolylines[j].empty() && lanePolylines[j].size() > 1) {
                    
                    // Calculate direction of first segment
                    cv::Point dir1 = lanePolylines[i].back() - lanePolylines[i].front();
                    
                    // Calculate direction of second segment
                    cv::Point dir2 = lanePolylines[j].back() - lanePolylines[j].front();
                    
                    // Calculate dot product to check direction similarity
                    float dotProduct = dir1.x * dir2.x + dir1.y * dir2.y;
                    similarDirection = (dotProduct > 0); // Positive dot product means similar direction
                }
                
                // Merge if horizontally close AND reasonable vertical gap AND similar direction
                if (hDist <= maxHorizontalDist && verticalGap <= maxVerticalGap && similarDirection) {
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
    for (auto& polyline : lanePolylines) {
        std::sort(polyline.begin(), polyline.end(), 
                 [](const cv::Point& a, const cv::Point& b) {
                     return a.y < b.y;
                 });
    }
}

void LaneDetector::drawPolyLanes(std::vector<std::vector<cv::Point>> lanePolylines, cv::Mat& allPolylinesViz) {
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),    // Blue
        cv::Scalar(0, 255, 0),    // Green
        cv::Scalar(0, 0, 255),    // Red
        cv::Scalar(255, 255, 0),  // Cyan
        cv::Scalar(255, 0, 255),  // Magenta
        cv::Scalar(0, 255, 255)   // Yellow
    };
    
    // Draw each polyline with a different color
    for (size_t i = 0; i < lanePolylines.size(); i++) {
        cv::Scalar color = colors[i % colors.size()];
        for (size_t j = 1; j < lanePolylines[i].size(); j++) {
            cv::line(allPolylinesViz, lanePolylines[i][j-1], lanePolylines[i][j], color, 2);
        }
        
        // Add a label for each polyline
        if (!lanePolylines[i].empty()) {
            std::string label = "Lane " + std::to_string(i+1);
            cv::putText(allPolylinesViz, label, lanePolylines[i][0], 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
    }
    
    // Display the number of polylines found
    std::string countText = "Polylines: " + std::to_string(lanePolylines.size());
    cv::putText(allPolylinesViz, countText, cv::Point(20, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

float LaneDetector::calculateLaneDistance(const std::vector<cv::Point>& lane1, 
                                        const std::vector<cv::Point>& lane2) {
    // Create normalized Y-position mapping of lane points
    std::map<int, cv::Point> lane1Points;
    std::map<int, cv::Point> lane2Points;
    
    // Normalize Y values to 0-100 range
    for (const auto& pt : lane1) {
        int normY = (pt.y * 100) / 480;  // Assuming 480 is max height
        lane1Points[normY] = pt;
    }
    
    for (const auto& pt : lane2) {
        int normY = (pt.y * 100) / 480;  // Assuming 480 is max height
        lane2Points[normY] = pt;
    }
    
    // Calculate average distance between lanes at matching Y positions
    float totalDist = 0;
    int matchCount = 0;
    
    for (const auto& p1 : lane1Points) {
        int y = p1.first;
        if (lane2Points.find(y) != lane2Points.end()) {
            // Calculate Euclidean distance
            float dist = cv::norm(p1.second - lane2Points[y]);
            totalDist += dist;
            matchCount++;
        }
    }
    
    return (matchCount > 0) ? (totalDist / matchCount) : FLT_MAX;
}
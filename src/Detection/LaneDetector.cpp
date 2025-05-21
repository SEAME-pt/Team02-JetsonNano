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
    float laneWidth = 1.0f;      // meters
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
    std::vector<std::vector<cv::Point>> lanePolylines = processLaneMask(binary_mask, 30, 40, 6);
    // std::cout << "Number of lane polylines after merging: " << lanePolylines.size() << std::endl;
    
    cv::Mat allPolylinesViz = frame.clone();
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

    float maxHorizontalDistance = frame.cols * 0.1; // 5% of frame width
    mergeLaneComponents(lanePolylines, maxHorizontalDistance, 0.0);

    if (lanePolylines.size() > 2) {
        // Sort by number of points (largest first)
        std::sort(lanePolylines.begin(), lanePolylines.end(), 
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return a.size() > b.size();
            });
        lanePolylines.resize(2);
    }

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
        
        // Determine left and right lanes based on the x-coordinate of lowest points
        // int centerX = frame.cols / 2;
        
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
    }

    allPolylinesViz.copyTo(frame);
}


std::vector<std::vector<cv::Point>> LaneDetector::processLaneMask(const cv::Mat& laneMask, int kernelSize, int minArea, int maxLanes) {    
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

void LaneDetector::mergeLaneComponents(std::vector<std::vector<cv::Point>>& lanePolylines, float maxHorizontalDist, float minOverlapRatio) {
    if (lanePolylines.size() <= 1) return;

    (void) minOverlapRatio;
    
    bool mergePerformed = true;
    while (mergePerformed) {
        mergePerformed = false;
        
        for (size_t i = 0; i < lanePolylines.size() && !mergePerformed; i++) {
            for (size_t j = i + 1; j < lanePolylines.size() && !mergePerformed; j++) {
                // Compute the y-range of both polylines
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
                
                // Check if they're horizontally aligned
                float hDist = std::abs(avgX1 - avgX2);
                
                // Check for vertical relationship (one above the other)
                bool verticallyAligned = (minY1 > maxY2) || (minY2 > maxY1);
                
                if (hDist <= maxHorizontalDist && verticallyAligned) {
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
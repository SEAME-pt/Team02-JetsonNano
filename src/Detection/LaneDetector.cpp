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
    // Step 1: Same as before - get connected components
    static cv::Mat verticalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize * 3));
    static cv::Mat horizontalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    
    cv::Mat result = laneMask.clone();
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, verticalKernel);
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, horizontalKernel);

    cv::Mat labels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(result, labels, stats, centroids, 8, CV_32S);
    
    // Step 2: Extract segments as before
    std::vector<std::pair<int, std::vector<cv::Point>>> segments;
    for (int i = 1; i < numLabels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > minArea) {
            std::vector<cv::Point> points;
            points.reserve(area/4);
            
            // Extract points for this component
            for (int y = 0; y < labels.rows; y += 2) {
                const int* row = labels.ptr<int>(y);
                int xStart = -1, xEnd = -1;
                
                for (int x = 0; x < labels.cols; x++) {
                    if (row[x] == i) {
                        if (xStart < 0) xStart = x;
                        xEnd = x;
                    }
                }
                
                if (xStart >= 0) {
                    int midX = (xStart + xEnd) / 2;
                    points.push_back(cv::Point(midX, y));
                }
            }
            
            if (!points.empty()) {
                segments.push_back({i, std::move(points)});
            }
        }
    }
    
    // Step 3: Calculate features for each segment
    std::vector<std::tuple<int, float, float, cv::Point>> segmentFeatures;
    for (const auto& segment : segments) {
        int id = segment.first;
        const auto& points = segment.second;
        
        // Calculate average x position
        float avgX = 0;
        for (const auto& pt : points) {
            avgX += pt.x;
        }
        avgX /= points.size();
        
        // Calculate angle (from linear regression or endpoints)
        cv::Point topPoint = points.front();
        cv::Point bottomPoint = points.back();
        float angle = atan2(bottomPoint.y - topPoint.y, bottomPoint.x - topPoint.x) * 180 / CV_PI;
        
        // Store segment features (id, avgX, angle, bottom point)
        segmentFeatures.push_back({id, avgX, angle, bottomPoint});
    }
    
    // Step 4: Cluster segments by x-position and angle
    std::vector<std::vector<int>> clusters;
    std::vector<bool> used(segments.size(), false);
    
    for (size_t i = 0; i < segments.size(); i++) {
        if (used[i]) continue;
        
        std::vector<int> currentCluster = {std::get<0>(segmentFeatures[i])};
        used[i] = true;
        
        for (size_t j = 0; j < segments.size(); j++) {
            if (used[j]) continue;
            
            float xDiff = abs(std::get<1>(segmentFeatures[i]) - std::get<1>(segmentFeatures[j]));
            float angleDiff = abs(std::get<2>(segmentFeatures[i]) - std::get<2>(segmentFeatures[j]));
            
            // If segments are similar in position and angle, cluster them
            if (xDiff < 30 && angleDiff < 20) {
                currentCluster.push_back(std::get<0>(segmentFeatures[j]));
                used[j] = true;
            }
        }
        
        clusters.push_back(currentCluster);
    }
    
    // Limit final clusters to maxLanes
    if (clusters.size() > static_cast<size_t>(maxLanes)) {
        // Sort clusters by average x-position
        std::sort(clusters.begin(), clusters.end(), 
                 [&segmentFeatures](const std::vector<int>& a, const std::vector<int>& b) {
                     float avgA = 0, avgB = 0;
                     for (int id : a) {
                         for (size_t i = 0; i < segmentFeatures.size(); i++) {
                             if (std::get<0>(segmentFeatures[i]) == id) {
                                 avgA += std::get<1>(segmentFeatures[i]);
                             }
                         }
                     }
                     for (int id : b) {
                         for (size_t i = 0; i < segmentFeatures.size(); i++) {
                             if (std::get<0>(segmentFeatures[i]) == id) {
                                 avgB += std::get<1>(segmentFeatures[i]);
                             }
                         }
                     }
                     avgA /= a.size();
                     avgB /= b.size();
                     return avgA < avgB;
                 });
        
        clusters.resize(maxLanes);
    }
    
    // Step 5: Create final lane polylines from merged clusters
    std::vector<std::vector<cv::Point>> lanePolylines;
    for (const auto& cluster : clusters) {
        std::vector<cv::Point> allPoints;
        
        for (int id : cluster) {
            for (const auto& segment : segments) {
                if (segment.first == id) {
                    allPoints.insert(allPoints.end(), segment.second.begin(), segment.second.end());
                    break;
                }
            }
        }
        
        // Sort points by y-position to create a coherent polyline
        std::sort(allPoints.begin(), allPoints.end(), 
                 [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });
        
        lanePolylines.push_back(std::move(allPoints));
    }
    
    return lanePolylines;
}
#include "ObjectDetector.hpp"
#include <sys/time.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace zenoh;

ObjectDetector::ObjectDetector(const std::string& enginePath,
                               std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    speed_lock_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/Speed/Lock")));

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
    cudaHostAlloc(&output_ptr, 10 * HEIGHT * WIDTH * sizeof(float),
                  cudaHostAllocMapped);
    inputData  = static_cast<float*>(input_ptr);
    outputData = static_cast<float*>(output_ptr);

    // Allocate GPU memory
    size_t pitch;
    cudaMallocPitch(&inputDevice, &pitch, WIDTH * sizeof(float),
                    HEIGHT * 3);
    cudaMallocPitch(&outputDevice, &pitch, WIDTH * sizeof(float),
                    HEIGHT * 10);

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

ObjectDetector::~ObjectDetector()
{
    cudaFreeHost(inputData);
    cudaFreeHost(outputData);
    cudaFree(inputDevice);
    cudaFree(outputDevice);
    cudaStreamDestroy(stream);
}

void ObjectDetector::createExecutionContext(const std::string& enginePath)
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

void ObjectDetector::detect(cv::Mat& frame)
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
                    10 * HEIGHT * WIDTH * sizeof(float),
                    cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

    // Postprocess
    postProcess(frame);

    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);

    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Inference time: " << milliseconds << "ms\n";
}

void ObjectDetector::preProcess(const cv::Mat& frame)
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

void ObjectDetector::postProcess(cv::Mat& frame)
{
    // Create a colored segmentation mask
    static cv::Mat colored_mask(HEIGHT, WIDTH, CV_8UC3);

    // Define color map for each class
    const cv::Scalar color_map[] = {
        cv::Scalar(0, 0, 0),      // Background
        cv::Scalar(128, 64, 128), // Road
        cv::Scalar(0, 0, 142),    // Car
        cv::Scalar(250, 170, 30), // Traffic Light
        cv::Scalar(220, 220, 0),  // Traffic Sign
        cv::Scalar(220, 20, 60),  // Person
        cv::Scalar(244, 35, 232), // Sidewalks
        cv::Scalar(0, 0, 70),     // Truck
        cv::Scalar(0, 60, 100),   // Bus
        cv::Scalar(0, 0, 230)     // Motorcycle
    };

    const int total_pixels = HEIGHT * WIDTH;

    // For each pixel, find the class with highest probability
    for (int i = 0; i < total_pixels; i++)
    {
        // Get probability for each class
        float probs[10];
        probs[0] = outputData[i];
        probs[1] = outputData[total_pixels * 1 + i];
        probs[2] = outputData[total_pixels * 2 + i];
        probs[3] = outputData[total_pixels * 3 + i];
        probs[4] = outputData[total_pixels * 4 + i];
        probs[5] = outputData[total_pixels * 5 + i];
        probs[6] = outputData[total_pixels * 6 + i];
        probs[7] = outputData[total_pixels * 7 + i];
        // probs[8] = outputData[total_pixels * 8 + i];
        // probs[9] = outputData[total_pixels * 9 + i];

        // Find class with highest probability
        int best_class = 0;
        float max_prob = probs[0];

        for (int c = 1; c < 8; c++)
        {
            if (probs[c] > max_prob)
            {
                max_prob   = probs[c];
                best_class = c;
            }
        }

        // Map pixel coordinates (i) back to x,y
        int y = i / WIDTH;
        int x = i % WIDTH;

        // Set pixel color based on class
        colored_mask.at<cv::Vec3b>(y, x) =
            cv::Vec3b(color_map[best_class][0], color_map[best_class][1],
                      color_map[best_class][2]);
    }

    // Resize the segmentation mask to match the frame size
    cv::Mat resized_mask;
    cv::resize(colored_mask, resized_mask, frame.size(), 0, 0,
               cv::INTER_NEAREST);

    // Check for collision in the danger zone
    bool collision_danger = checkForwardCollision(colored_mask);

    if (collision_danger)
    {
        cv::putText(frame, "OBSTACLE DETECTED!",
                    cv::Point(frame.cols / 2 - 150, frame.rows / 2),
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
            std::cerr << "Error sending CAN message on Object Detector: " << e.what()
                      << std::endl;
        }

    }
    else if (is_emergency_stop)
    {
        std::cout << "\033[1;32m*** PATH CLEAR - READY TO RESUME ***\033[0m"
                  << std::endl;
        is_emergency_stop = false;

        publishSpeedLock("0");
    }

    cv::addWeighted(frame, 0.7, resized_mask, 0.3, 0, frame);
}

bool ObjectDetector::checkForwardCollision(const cv::Mat& segmentation_mask)
{
    // Define danger zone (lower-center portion of the image)
    const int zone_width  = WIDTH * 0.6;              // 60% of image width
    const int zone_height = HEIGHT * 0.4;             // 40% of image height
    const int zone_x      = (WIDTH - zone_width) / 2; // Center horizontally
    const int zone_y      = HEIGHT - zone_height;     // Bottom of image

    // Count pixels in danger zone by class
    int total_pixels = 0;
    int road_pixels  = 0;

    for (int y = zone_y; y < HEIGHT; y++)
    {
        for (int x = zone_x; x < zone_x + zone_width; x++)
        {
            total_pixels++;
            cv::Vec3b pixel = segmentation_mask.at<cv::Vec3b>(y, x);

            if (pixel == cv::Vec3b(128, 64, 128))
            {
                road_pixels++;
            }
        }
    }

    // Calculate percentage of road in danger zone
    float road_percentage = static_cast<float>(road_pixels) / total_pixels;

    const float SAFE_ROAD_THRESHOLD = 0.7; // 70% of zone should be road
    bool danger_detected            = (road_percentage < SAFE_ROAD_THRESHOLD);

    // Draw danger zone on frame (green if safe, red if danger)
    cv::Scalar zone_color =
        danger_detected ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    cv::rectangle(segmentation_mask,
                  cv::Rect(zone_x, zone_y, zone_width, zone_height), zone_color,
                  2);

    return danger_detected;
}

void ObjectDetector::publishSpeedLock(const std::string &value_str)
{
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    speed_lock_publisher_->put(std::move(buf));
}
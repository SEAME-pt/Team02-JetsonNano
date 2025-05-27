#include "ObjectDetector.hpp"

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

    try {
        this->gpuInference = new GPUInference(enginePath, 3, 7);
        this->gpuInference->init(); 
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing GPUInference" << e.what() << std::endl;
    }

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
    delete gpuInference;
    delete canBus;
}

void ObjectDetector::detect(cv::Mat& frame)
{
    cv::Mat class_mask(HEIGHT, WIDTH, CV_8UC3);
    cv::Mat preprocessedFrame(HEIGHT, WIDTH, CV_8UC3);
    
    preProcess(frame, preprocessedFrame);

    gpuInference->copyToGPU(preprocessedFrame);
    gpuInference->inference();
    gpuInference->copyToCPUClassOutput(class_mask);
    
    postProcess(frame, class_mask);
}

void ObjectDetector::preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame)
{
    // Create static GPU matrices
    static cv::cuda::GpuMat d_frame, d_resized, d_rgb_image;

    // Upload input frame to GPU
    d_frame.upload(frame);

    // Resize on GPU with CUDA stream
    cv::cuda::resize(d_frame, d_resized, cv::Size(WIDTH, HEIGHT), 0, 0,
                     cv::INTER_NEAREST, cv_stream);

    // Convert BGR to RGB on GPU
    cv::cuda::cvtColor(d_resized, d_rgb_image, cv::COLOR_BGR2RGB, 0, cv_stream);

    // Download the result back to CPU
    d_rgb_image.download(preprocessedFrame, cv_stream);

    // Wait for CUDA operations to complete
    cv_stream.waitForCompletion();
}

void ObjectDetector::postProcess(cv::Mat& frame, cv::Mat& class_mask)
{
    // Resize the segmentation mask to match the frame size
    cv::Mat resized_mask;
    cv::resize(class_mask, resized_mask, frame.size(), 0, 0,
               cv::INTER_NEAREST);

    // Check for collision in the danger zone
    bool collision_danger = checkForwardCollision(resized_mask);

    if (collision_danger)
    {
        cv::putText(frame, "OBSTACLE DETECTED!",
                    cv::Point(frame.cols / 2 - 150, frame.rows / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);

        std::cout << "\033[1;31m*** WARNING: OBSTACLE DETECTED! STOPPING "
                     "VEHICLE ***\033[0m"
                  << std::endl;

        is_emergency_stop = true;

        // publishSpeedLock("1");
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
    const int zone_height = HEIGHT * 0.6;             // 40% of image height
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
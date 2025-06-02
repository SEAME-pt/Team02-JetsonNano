#include "ObjectDetector.hpp"

using namespace cv;
using namespace std;
using namespace zenoh;

ObjectDetector::ObjectDetector(const std::string& enginePath)
{
    try
    {
        this->gpuInference = new GPUInference(enginePath, 3, 7);
        this->gpuInference->init();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing GPUInference" << e.what() << std::endl;
    }
}

ObjectDetector::~ObjectDetector()
{
    delete gpuInference;
}

void ObjectDetector::detect(cv::Mat& frame, cv::Mat& result)
{
    cv::Mat class_mask(HEIGHT, WIDTH, CV_8UC3);
    cv::Mat preprocessedFrame(HEIGHT, WIDTH, CV_8UC3);

    preProcess(frame, preprocessedFrame);

    gpuInference->copyToGPU(preprocessedFrame);
    gpuInference->inference();
    gpuInference->copyToCPUClassOutput(class_mask);

    class_mask.copyTo(result);
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
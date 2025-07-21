#include "LaneDetector.hpp"
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

LaneDetector::LaneDetector(const std::string& enginePath, int height, int width)
    : height_(height), width_(width)
{
    try
    {
        this->gpuInference =
            new GPUInference(enginePath, 3, 1, height_, width_);
        this->gpuInference->init();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing GPUInference" << e.what() << std::endl;
        throw std::runtime_error("Error initializing GPUInference");
    }

    // cv_stream = cv::cuda::Stream();
}

LaneDetector::~LaneDetector()
{
    delete gpuInference;
}

void LaneDetector::detect(cv::Mat& frame, cv::Mat& result)
{
    cv::Mat binary_mask(height_, width_, CV_8UC1);
    cv::Mat preprocessedFrame(height_, width_, CV_8UC3);

    preProcess(frame, preprocessedFrame);

    gpuInference->copyToGPU(preprocessedFrame);
    gpuInference->inference();
    gpuInference->copyToCPUBinaryOutput(binary_mask);

    binary_mask.copyTo(result);
}

void LaneDetector::preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame)
{
    cv::Mat resized;

    cv::resize(frame, resized, cv::Size(width_, height_), 0, 0,
               cv::INTER_LINEAR);

    cv::cvtColor(resized, preprocessedFrame, cv::COLOR_BGR2RGB);
}

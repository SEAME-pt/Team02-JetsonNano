#include "LaneDetector.hpp"
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

LaneDetector::LaneDetector(const std::string& enginePath)
{
    try
    {
        this->gpuInference = new GPUInference(enginePath, 3, 1);
        this->gpuInference->init();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing GPUInference" << e.what() << std::endl;
    }

    cv_stream = cv::cuda::Stream();
}

LaneDetector::~LaneDetector()
{
    delete gpuInference;
}

void LaneDetector::detect(cv::Mat& frame, cv::Mat& result)
{
    cv::Mat binary_mask(HEIGHT, WIDTH, CV_8UC1);
    cv::Mat preprocessedFrame(HEIGHT, WIDTH, CV_8UC3);

    preProcess(frame, preprocessedFrame);

    gpuInference->copyToGPU(preprocessedFrame);
    gpuInference->inference();
    gpuInference->copyToCPUBinaryOutput(binary_mask);

    binary_mask.copyTo(result);
}

void LaneDetector::preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame)
{
    static cv::cuda::GpuMat d_frame, d_resized, d_rgb_image;

    d_frame.upload(frame);

    cv::cuda::resize(d_frame, d_resized, cv::Size(WIDTH, HEIGHT), 0, 0,
                     cv::INTER_NEAREST, cv_stream);

    cv::cuda::cvtColor(d_resized, d_rgb_image, cv::COLOR_BGR2RGB, 0, cv_stream);

    d_rgb_image.download(preprocessedFrame, cv_stream);

    cv_stream.waitForCompletion();
}

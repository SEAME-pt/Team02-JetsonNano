#include "ObjectDetector.hpp"

using namespace cv;
using namespace std;
using namespace zenoh;

ObjectDetector::ObjectDetector(const std::string& enginePath, int height,
                               int width)
    : height_(height), width_(width)
{
    try
    {
        this->gpuInference =
            new GPUInference(enginePath, 3, 8, height_, width_);
        this->gpuInference->init();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing GPUInference" << e.what() << std::endl;
        throw std::runtime_error("Error initializing GPUInference");
    }
}

ObjectDetector::~ObjectDetector()
{
    delete gpuInference;
}

void ObjectDetector::detect(cv::Mat& frame, cv::Mat& result)
{
    cv::Mat class_mask(height_, width_, CV_8UC3);
    cv::Mat preprocessedFrame(height_, width_, CV_8UC3);

    preProcess(frame, preprocessedFrame);

    gpuInference->copyToGPU(preprocessedFrame);
    gpuInference->inference();
    gpuInference->copyToCPUClassOutput(class_mask);

    class_mask.copyTo(result);
}

void ObjectDetector::preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame)
{
    cv::Mat resized;

    cv::resize(frame, resized, cv::Size(width_, height_), 0, 0,
               cv::INTER_LINEAR);

    cv::cvtColor(resized, preprocessedFrame, cv::COLOR_BGR2RGB);
}
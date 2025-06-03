#include "ObjectDetector.hpp"

using namespace cv;
using namespace std;
using namespace zenoh;

ObjectDetector::ObjectDetector(const std::string& enginePath)
{
    try {
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
    cv::Mat resized;
    
    cv::resize(frame, resized, cv::Size(WIDTH, HEIGHT), 0, 0, cv::INTER_NEAREST);
    
    cv::cvtColor(resized, preprocessedFrame, cv::COLOR_BGR2RGB);
}
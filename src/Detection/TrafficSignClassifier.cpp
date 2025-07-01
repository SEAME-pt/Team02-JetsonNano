#include "TrafficSignClassifier.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>

TrafficSignClassifier::TrafficSignClassifier(const std::string& enginePath, int height, int width)
    : height_(height), width_(width)
{
    try {
        this->gpuInference = new GPUInference(enginePath, 3, 7, height_, width_);
        this->gpuInference->init();
    } catch (const std::exception& e) {
        std::cerr << "Error initializing GPUInference: " << e.what() << std::endl;
        throw std::runtime_error("Error initializing GPUInference");
    }
}

TrafficSignClassifier::~TrafficSignClassifier()
{
    delete gpuInference;
}

void TrafficSignClassifier::classify(cv::Mat& frame, cv::Mat& class_mask)
{
    // cv::Mat class_mask(height_, width_, CV_8UC3);
    // cv::Mat preprocessedFrame(height_, width_, CV_8UC3);

    // preProcess(frame, preprocessedFrame);

    // gpuInference->copyToGPU(preprocessedFrame);
    // gpuInference->inference();
    // gpuInference->copyToCPUTrafficOutput(class_mask);

    (void) frame;

    // 1. Create a binary mask for (220, 220, 0)
    cv::Mat binary_mask;
    cv::inRange(class_mask, cv::Scalar(220, 220, 0), cv::Scalar(220, 220, 0), binary_mask);

    // 2. Find connected components (blocks)
    cv::Mat labels;
    int nLabels = cv::connectedComponents(binary_mask, labels, 8, CV_32S);

    // nLabels includes background (label 0), so subtract 1
    int numBlocks = nLabels - 1;

    std::cout << "Detected " << numBlocks << " blocks of (220, 220, 0) in class_mask." << std::endl;

}

void TrafficSignClassifier::preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame)
{
    cv::Mat resized;
    
    cv::resize(frame, resized, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);
    
    cv::cvtColor(resized, preprocessedFrame, cv::COLOR_BGR2RGB);
}
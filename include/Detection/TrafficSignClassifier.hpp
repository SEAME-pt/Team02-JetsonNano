#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include "GPUInference.hpp"

class TrafficSignClassifier {
public:
    TrafficSignClassifier(const std::string& enginePath, int height, int width);
    ~TrafficSignClassifier();

    void classify(cv::Mat& frame, cv::Mat& class_mask);
    
    void preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame);

private:
    GPUInference* gpuInference;
    int inputSize_;
};
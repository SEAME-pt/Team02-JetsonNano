#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include "GPUInference.hpp"

class TrafficSignClassifier {
public:
    TrafficSignClassifier(const std::string& enginePath, int inputSize = 30);
    ~TrafficSignClassifier();

    // Each roi is a cv::Rect(x, y, size, size)
    void classify(const cv::Mat& frame,
                  const std::vector<cv::Rect>& rois,
                  std::vector<int>& class_ids,
                  std::vector<float>& confidences);

private:
    GPUInference* gpuInference;
    int inputSize_;
};
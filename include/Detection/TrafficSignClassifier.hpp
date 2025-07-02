#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include "GPUInference.hpp"

class TrafficSignClassifier {
public:
    TrafficSignClassifier(const std::string& enginePath, std::shared_ptr<zenoh::Session> session, int height, int width);
    ~TrafficSignClassifier();

    void classify(cv::Mat frame, cv::Mat& class_mask, cv::Mat& result);

    void publishTrafficSignFrame(const std::string& value_str);
    void publishTrafficSign(const std::string& value_str);

private:
    void preProcess(cv::Mat frame, cv::Mat& preprocessedFrame);

private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;
    std::optional<zenoh::Publisher> trafficSign_mask_publisher_;
    std::optional<zenoh::Publisher> trafficSign_publisher_;

    GPUInference* gpuInference;
    int height_;
    int width_;
};
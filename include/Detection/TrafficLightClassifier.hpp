#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include "GPUInference.hpp"

class TrafficLightClassifier {
public:
    TrafficLightClassifier(std::shared_ptr<zenoh::Session> session);
    ~TrafficLightClassifier();

    void classify(cv::Mat frame, cv::Mat& class_mask, cv::Mat& result);

    void publishTrafficLightFrame(const std::string& value_str);
    void publishTrafficLight(const std::string& value_str);

private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;
    std::optional<zenoh::Publisher> trafficLight_mask_publisher_;
    std::optional<zenoh::Publisher> trafficLight_publisher_;
};
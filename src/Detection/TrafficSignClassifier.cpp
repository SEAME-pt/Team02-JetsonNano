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
    std::cout "Classification ----------------------------------" << std::endl;
    // cv::Mat class_mask(height_, width_, CV_8UC3);
    // cv::Mat preprocessedFrame(height_, width_, CV_8UC3);

    // preProcess(frame, preprocessedFrame);

    // gpuInference->copyToGPU(preprocessedFrame);
    // gpuInference->inference();
    // gpuInference->copyToCPUTrafficOutput(class_mask);

    // class_ids.clear();
    // confidences.clear();

    // for (const auto& roi : rois) {
    //     // Crop and resize ROI
    //     cv::Rect clipped_roi = roi & cv::Rect(0, 0, frame.cols, frame.rows);
    //     if (clipped_roi.width <= 0 || clipped_roi.height <= 0) {
    //         class_ids.push_back(-1);
    //         confidences.push_back(0.0f);
    //         continue;
    //     }
    //     cv::Mat sign = frame(clipped_roi).clone();
    //     cv::resize(sign, sign, cv::Size(inputSize_, inputSize_));

    //     // Preprocess: BGR to RGB, float, normalization (update mean/std as needed)
    //     cv::cvtColor(sign, sign, cv::COLOR_BGR2RGB);
    //     sign.convertTo(sign, CV_32FC3, 1.0 / 255.0);
    //     cv::Scalar mean(0.485, 0.456, 0.406);
    //     cv::Scalar std(0.229, 0.224, 0.225);
    //     sign = (sign - mean) / std;

    //     // Run inference
    //     gpuInference->copyToGPU(sign);
    //     gpuInference->inference();
    //     std::vector<float> output;
    //     gpuInference->copyToCPUClassOutput(output);

    //     // Softmax
    //     float max_val = *std::max_element(output.begin(), output.end());
    //     float sum = 0.0f;
    //     for (auto& v : output) {
    //         v = std::exp(v - max_val);
    //         sum += v;
    //     }
    //     for (auto& v : output) v /= sum;

    //     // Get class and confidence
    //     auto max_it = std::max_element(output.begin(), output.end());
    //     int class_id = std::distance(output.begin(), max_it);
    //     float confidence = *max_it;

    //     class_ids.push_back(class_id);
    //     confidences.push_back(confidence);
    // }
}

void TrafficSignClassifier::preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame)
{
    cv::Mat resized;
    
    cv::resize(frame, resized, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);
    
    cv::cvtColor(resized, preprocessedFrame, cv::COLOR_BGR2RGB);
}
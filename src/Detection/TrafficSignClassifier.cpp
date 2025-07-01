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

    cv::Mat binary_mask;
    cv::inRange(class_mask, cv::Scalar(220, 220, 0), cv::Scalar(220, 220, 0), binary_mask);

    cv::Mat labels;
    int nLabels = cv::connectedComponents(binary_mask, labels, 8, CV_32S);

    int numBlocks = nLabels - 1;

    std::cout << "Detected " << numBlocks << " blocks of (220, 220, 0) in class_mask." << std::endl;

    // Count the size of each component (excluding background label 0)
    std::vector<int> componentSizes(nLabels, 0);
    for (int y = 0; y < labels.rows; ++y) {
        for (int x = 0; x < labels.cols; ++x) {
            int label = labels.at<int>(y, x);
            if (label > 0) {
                componentSizes[label]++;
            }
        }
    }

    // Print the size of each component
    for (int i = 1; i < nLabels; ++i) {
        std::cout << "Block " << i << " size: " << componentSizes[i] << " pixels" << std::endl;
    }

}

void TrafficSignClassifier::preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame)
{
    cv::Mat resized;
    
    cv::resize(frame, resized, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);
    
    cv::cvtColor(resized, preprocessedFrame, cv::COLOR_BGR2RGB);
}
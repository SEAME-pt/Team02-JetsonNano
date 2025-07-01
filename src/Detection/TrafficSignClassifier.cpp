#include "TrafficSignClassifier.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>

TrafficSignClassifier::TrafficSignClassifier(const std::string& enginePath, 
    std::shared_ptr<zenoh::Session> session, int height, int width)
    : height_(height), width_(width)
{
    try {
        this->gpuInference = new GPUInference(enginePath, 3, 7, height_, width_);
        this->gpuInference->init();
    } catch (const std::exception& e) {
        std::cerr << "Error initializing GPUInference: " << e.what() << std::endl;
        throw std::runtime_error("Error initializing GPUInference");
    }

    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    trafficSign_mask_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/TrafficMask")));
}

TrafficSignClassifier::~TrafficSignClassifier()
{
    delete gpuInference;
}

void TrafficSignClassifier::classify(cv::Mat& frame, cv::Mat& class_mask, cv::Mat& result)
{
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

    for (int i = 1; i < nLabels; ++i) {
        if (componentSizes[i] >= 1000) {
            int minX = labels.cols, minY = labels.rows, maxX = 0, maxY = 0;
            // Find bounding box for this component
            for (int y = 0; y < labels.rows; ++y) {
                for (int x = 0; x < labels.cols; ++x) {
                    if (labels.at<int>(y, x) == i) {
                        minX = std::min(minX, x);
                        minY = std::min(minY, y);
                        maxX = std::max(maxX, x);
                        maxY = std::max(maxY, y);
                    }
                }
            }
            // Expand bounding box by a margin (e.g., 10 pixels)
            int margin = 10;
            minX = std::max(0, minX - margin);
            minY = std::max(0, minY - margin);
            maxX = std::min(labels.cols - 1, maxX + margin);
            maxY = std::min(labels.rows - 1, maxY + margin);

            // Crop from the original frame (or class_mask)
            cv::Rect roi(minX, minY, maxX - minX + 1, maxY - minY + 1);
            cv::Mat cropped = frame(roi).clone(); // or class_mask(roi).clone();

            std::cout << "Block " << i << " size: " << componentSizes[i]
                    << " pixels, cropped region: " << roi << std::endl;
        }
    }

    cv::Mat preprocessedFrame(height_, width_, CV_8UC3);

    preProcess(cropped, preprocessedFrame);

    gpuInference->copyToGPU(preprocessedFrame);
    gpuInference->inference();

    cropped.copyTo(result);

    // gpuInference->copyToCPUTrafficOutput(class_mask);
}

void TrafficSignClassifier::preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame)
{
    cv::Mat resized;
    
    cv::resize(frame, resized, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);
    
    cv::cvtColor(resized, preprocessedFrame, cv::COLOR_BGR2RGB);
}

void TrafficSignClassifier::publishTrafficSignFrame(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    trafficSign_mask_publisher_->put(std::move(buf));
}
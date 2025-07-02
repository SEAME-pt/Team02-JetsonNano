#include "TrafficLightClassifier.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>

TrafficLightClassifier::TrafficLightClassifier(std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    trafficLight_mask_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/TrafficLightMask")));
    trafficLight_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/TrafficLight")));
}

TrafficLightClassifier::~TrafficLightClassifier()
{
}

void TrafficLightClassifier::classify(cv::Mat frame, cv::Mat& class_mask, cv::Mat& result)
{
    std::vector<cv::Mat> croppedBlocks;

    cv::Mat resized_class_mask;
    cv::resize(class_mask, resized_class_mask, frame.size(), 0, 0, cv::INTER_LINEAR);

    cv::Mat binary_mask;
    cv::inRange(resized_class_mask, cv::Scalar(250, 0, 0), cv::Scalar(250, 0, 0), binary_mask);

    cv::Mat labels;
    int nLabels = cv::connectedComponents(binary_mask, labels, 8, CV_32S);

    std::cout << nLabels - 1 << std::endl;
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
        if (componentSizes[i] >= 5) {
            std::cout << "Here" << std::endl;
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
            int margin = 1;
            minX = std::max(0, minX - margin);
            minY = std::max(0, minY - margin);
            maxX = std::min(labels.cols - 1, maxX + margin);
            maxY = std::min(labels.rows - 1, maxY + margin);

            // Crop from the original frame (or class_mask)
            cv::Rect roi(minX, minY, maxX - minX + 1, maxY - minY + 1);
            cv::Mat cropped = frame(roi).clone();

            const int fixed_width = 64;
            const int fixed_height = 64;
            const int fixed_type = CV_8UC3;
            if (!cropped.empty()) {
                cv::Mat resized_cropped;
                cv::resize(cropped, resized_cropped, cv::Size(fixed_width, fixed_height));
                if (resized_cropped.type() != fixed_type) {
                    resized_cropped.convertTo(resized_cropped, fixed_type);
                }
                croppedBlocks.push_back(resized_cropped);
            }
        }
    }
    // Concatenate all cropped blocks vertically
    if (!croppedBlocks.empty()) {
        cv::vconcat(croppedBlocks, result);
    } else {
        result = cv::Mat();
    }
}
void TrafficLightClassifier::publishTrafficLightFrame(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    trafficLight_publisher_->put(std::move(buf));
}

void TrafficLightClassifier::publishTrafficLight(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    trafficLight_publisher_->put(std::move(buf));
}
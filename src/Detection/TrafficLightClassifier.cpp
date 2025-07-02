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

std::string detectTrafficLightColor(const cv::Mat& img, float ratio = 1.2, int min_brightness = 3000) {
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // Red mask (two ranges)
    cv::Mat mask_red1, mask_red2, red_mask, yellow_mask, green_mask;
    cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask_red1);
    cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask_red2);
    cv::add(mask_red1, mask_red2, red_mask);

    // Yellow mask
    cv::inRange(hsv, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), yellow_mask);

    // Green mask
    cv::inRange(hsv, cv::Scalar(40, 40, 40), cv::Scalar(90, 255, 255), green_mask);

    int red_brightness = cv::sum(red_mask)[0];
    int yellow_brightness = cv::sum(yellow_mask)[0];
    int green_brightness = cv::sum(green_mask)[0];

    std::vector<int> brightnesses = {red_brightness, yellow_brightness, green_brightness};
    std::vector<std::string> color_names = {"Red", "Yellow", "Green"};

    int max_idx = std::distance(brightnesses.begin(), std::max_element(brightnesses.begin(), brightnesses.end()));
    int max_value = brightnesses[max_idx];
    std::vector<int> other_vals;
    for (int i = 0; i < 3; ++i) if (i != max_idx) other_vals.push_back(brightnesses[i]);

    std::string detected = "None";
    if (max_value > min_brightness &&
        std::all_of(other_vals.begin(), other_vals.end(), [max_value, ratio](int val) { return max_value > ratio * val; })) {
        detected = color_names[max_idx];
    }

    return detected;
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
        if (componentSizes[i] >= 500) {
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
            int margin = 10;
            minX = std::max(0, minX - margin);
            minY = std::max(0, minY - margin);
            maxX = std::min(labels.cols - 1, maxX + margin);
            maxY = std::min(labels.rows - 1, maxY + margin);

            // Crop from the original frame (or class_mask)
            cv::Rect roi(minX, minY, maxX - minX + 1, maxY - minY + 1);
            cv::Mat cropped = frame(roi).clone();

            const int fixed_width = 32;
            const int fixed_height = 64;
            const int fixed_type = CV_8UC3;
            if (!cropped.empty()) {
                std::string color = detectTrafficLightColor(cropped);

                std::cout << "Color: " << color << std::endl;

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
    trafficLight_mask_publisher_->put(std::move(buf));
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
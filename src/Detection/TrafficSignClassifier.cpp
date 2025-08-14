#include "TrafficSignClassifier.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>

TrafficSignClassifier::TrafficSignClassifier(
    const std::string& enginePath, std::shared_ptr<zenoh::Session> session,
    int height, int width)
    : height_(height), width_(width)
{
    try
    {
        this->gpuInference =
            new GPUInference(enginePath, 3, 10, height_, width_);
        this->gpuInference->init();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing GPUInference: " << e.what()
                  << std::endl;
        throw std::runtime_error("Error initializing GPUInference");
    }

    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));
    trafficSign_mask_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/TrafficMask")));
    trafficSign_publisher_.emplace(
        session_->declare_publisher(zenoh::KeyExpr("Vehicle/1/TrafficSign")));
}

TrafficSignClassifier::~TrafficSignClassifier()
{
    delete gpuInference;
}

void TrafficSignClassifier::classify(cv::Mat frame, cv::Mat& class_mask,
                                     cv::Mat& result)
{
    std::vector<cv::Mat> croppedBlocks;

    cv::Mat resized_class_mask;
    cv::resize(class_mask, resized_class_mask, frame.size(), 0, 0,
               cv::INTER_LINEAR);

    cv::Mat mask1, mask2, binary_mask;
    cv::inRange(resized_class_mask, cv::Scalar(220, 220, 0),
                cv::Scalar(220, 220, 0), mask1); // yellow
    cv::inRange(resized_class_mask, cv::Scalar(250, 0, 0),
                cv::Scalar(250, 0, 0), mask2); // red
    cv::bitwise_or(mask1, mask2, binary_mask);

    cv::Mat labels;
    int nLabels = cv::connectedComponents(binary_mask, labels, 8, CV_32S);

    std::vector<int> componentSizes(nLabels, 0);
    for (int y = 0; y < labels.rows; ++y)
    {
        for (int x = 0; x < labels.cols; ++x)
        {
            int label = labels.at<int>(y, x);
            if (label > 0)
            {
                componentSizes[label]++;
            }
        }
    }

    for (int i = 1; i < nLabels; ++i)
    {
        if (componentSizes[i] >= 500)
        {
            int minX = labels.cols, minY = labels.rows, maxX = 0, maxY = 0;
            // Find bounding box for this component
            for (int y = 0; y < labels.rows; ++y)
            {
                for (int x = 0; x < labels.cols; ++x)
                {
                    if (labels.at<int>(y, x) == i)
                    {
                        minX = std::min(minX, x);
                        minY = std::min(minY, y);
                        maxX = std::max(maxX, x);
                        maxY = std::max(maxY, y);
                    }
                }
            }
            int margin = 5;
            minX       = std::max(0, minX - margin);
            minY       = std::max(0, minY - margin);
            maxX       = std::min(labels.cols - 1, maxX + margin);
            maxY       = std::min(labels.rows - 1, maxY + margin);

            // Crop from the original frame (or class_mask)
            cv::Rect roi(minX, minY, maxX - minX + 1, maxY - minY + 1);
            cv::Mat cropped = frame(roi).clone();

            if (!cropped.empty())
            {
                cv::Mat preprocessedFrame(height_, width_, CV_8UC3);
                preProcess(cropped, preprocessedFrame);

                gpuInference->copyToGPU(preprocessedFrame);
                gpuInference->inference();
                int bestClass = gpuInference->copyToCPUTrafficOutput();

                if (bestClass != -1)
                {
                    static const std::string classes[10] = {
                        "Speed 50km/h",  "Speed 80km/h", "Yield",
                        "Stop",          "Danger",       "Crosswalk",
                        "Traffic Green", "Traffic Red",  "Traffic Yellow",
                        "Unknown"};

                    if (classes[bestClass].find("Stop") == std::string::npos) {
                        publishTrafficSign(classes[bestClass]);
                    } else {
                        if (componentSizes[i] > 1000)
                        {
                            publishTrafficSign(classes[bestClass]);
                        }
                    }


                    cv::putText(preprocessedFrame, classes[bestClass],
                                cv::Point(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.3,
                                cv::Scalar(0, 255, 0), 0.4);
                }

                croppedBlocks.push_back(preprocessedFrame);
            }
        }
    }
    // Concatenate all cropped blocks vertically
    if (!croppedBlocks.empty())
    {
        cv::vconcat(croppedBlocks, result);
    }
    else
    {
        result = cv::Mat();
    }
}

void TrafficSignClassifier::preProcess(cv::Mat frame,
                                       cv::Mat& preprocessedFrame)
{
    cv::Mat resized;

    cv::resize(frame, resized, cv::Size(width_, height_), 0, 0,
               cv::INTER_LINEAR);

    cv::cvtColor(resized, preprocessedFrame, cv::COLOR_BGR2RGB);
}

void TrafficSignClassifier::publishTrafficSignFrame(
    const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    trafficSign_mask_publisher_->put(std::move(buf));
}

void TrafficSignClassifier::publishTrafficSign(const std::string& value_str)
{
    const auto len = value_str.size() + 1;
    auto alloc_result =
        provider_->alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    trafficSign_publisher_->put(std::move(buf));
}
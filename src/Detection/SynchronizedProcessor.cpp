#include "SynchronizedProcessor.hpp"

SynchronizedProcessor::SynchronizedProcessor()
{
    std::lock_guard<std::mutex> lock(sync_mutex);
    current_frame = cv::Mat();
}

SynchronizedProcessor::~SynchronizedProcessor() {}

void SynchronizedProcessor::setNewFrame(const cv::Mat& frame)
{
    std::unique_lock<std::mutex> lock(sync_mutex);

    camera_cv.wait_for(
        lock, std::chrono::milliseconds(100),
        [this]() { return lane_ready && object_ready && trajectory_done; });

    frame.copyTo(current_frame);

    lane_ready          = false;
    object_ready        = false;
    new_frame_available = true;

    inference_cv.notify_all();
}

cv::Mat SynchronizedProcessor::getLaneFrame()
{
    std::unique_lock<std::mutex> lock(sync_mutex);

    inference_cv.wait_for(lock, std::chrono::milliseconds(100), [this]()
                          { return new_frame_available && !lane_ready; });

    return current_frame.clone();
}

cv::Mat SynchronizedProcessor::getObjectFrame()
{
    std::unique_lock<std::mutex> lock(sync_mutex);

    inference_cv.wait_for(lock, std::chrono::milliseconds(100), [this]()
                          { return new_frame_available && !object_ready; });

    return current_frame.clone();
}

void SynchronizedProcessor::laneDone(const cv::Mat& result)
{
    std::unique_lock<std::mutex> lock(sync_mutex);

    result.copyTo(lane_binary_mask);
    lane_ready = true;

    checkBothDone();
}

void SynchronizedProcessor::objectDone(const cv::Mat& result)
{
    std::unique_lock<std::mutex> lock(sync_mutex);

    result.copyTo(object_class_mask);
    object_ready = true;

    checkBothDone();
}

void SynchronizedProcessor::getProcessingData(cv::Mat& original,
                                              cv::Mat& lane_mask,
                                              cv::Mat& object_mask)
{
    std::unique_lock<std::mutex> lock(sync_mutex);

    trajectory_cv.wait_for(
        lock, std::chrono::milliseconds(100),
        [this]() { return lane_ready && object_ready && !trajectory_done; });

    current_frame.copyTo(original);
    lane_binary_mask.copyTo(lane_mask);
    object_class_mask.copyTo(object_mask);
}

void SynchronizedProcessor::getFrameAndObjectMask(cv::Mat& frame,
                                                  cv::Mat& object_mask)
{
    std::unique_lock<std::mutex> lock(sync_mutex);

    inference_cv.wait_for(lock, std::chrono::milliseconds(100), [this]()
                          { return new_frame_available && object_ready; });

    current_frame.copyTo(frame);
    object_class_mask.copyTo(object_mask);
}

void SynchronizedProcessor::trajectoryDone()
{
    std::unique_lock<std::mutex> lock(sync_mutex);
    trajectory_done = true;

    camera_cv.notify_one();
}

void SynchronizedProcessor::checkBothDone()
{
    if (lane_ready && object_ready)
    {
        trajectory_done     = false;
        new_frame_available = false;
        trajectory_cv.notify_one();
    }
}

void SynchronizedProcessor::shutdown()
{
    {
        std::unique_lock<std::mutex> lock(sync_mutex);
        lane_ready          = true;
        object_ready        = true;
        trajectory_done     = true;
        new_frame_available = true;

        inference_cv.notify_all();
        trajectory_cv.notify_all();
        camera_cv.notify_all();
    }
}
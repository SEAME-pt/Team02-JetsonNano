#pragma once

#include <condition_variable>
#include <queue>             
#include <memory>            

class SynchronizedProcessor
{
  private:
    std::mutex sync_mutex;
    std::condition_variable inference_cv;
    std::condition_variable trajectory_cv;
    std::condition_variable camera_cv;

    cv::Mat current_frame;
    cv::Mat lane_binary_mask;
    cv::Mat object_class_mask;

    bool lane_ready          = true;
    bool object_ready        = true;
    bool trajectory_done     = true;
    bool new_frame_available = false;
    int frame_id             = 0;

  public:
    void setNewFrame(const cv::Mat& frame)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        camera_cv.wait(
            lock,
            [this]() { return lane_ready && object_ready && trajectory_done; });

        frame.copyTo(current_frame);

        lane_ready          = false;
        object_ready        = false;
        new_frame_available = true;

        inference_cv.notify_all();
    }

    // Lane detection thread calls this
    cv::Mat getLaneFrame()
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        inference_cv.wait(lock, [this]()
                          { return new_frame_available && !lane_ready; });

        return current_frame.clone();
    }

    cv::Mat getObjectFrame()
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        inference_cv.wait(lock, [this]()
                          { return new_frame_available && !object_ready; });

        return current_frame.clone();
    }

    void laneDone(const cv::Mat& result)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        result.copyTo(lane_binary_mask);
        lane_ready = true;

        checkBothDone();
    }

    // Object thread calls this when done
    void objectDone(const cv::Mat& result)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        result.copyTo(object_class_mask);
        object_ready = true;

        checkBothDone();
    }

    void getProcessingData(cv::Mat& original, cv::Mat& lane_mask,
                           cv::Mat& object_mask)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        trajectory_cv.wait(
            lock, [this]()
            { return lane_ready && object_ready && !trajectory_done; });

        current_frame.copyTo(original);
        lane_binary_mask.copyTo(lane_mask);
        object_class_mask.copyTo(object_mask);
    }

    void trajectoryDone()
    {
        std::unique_lock<std::mutex> lock(sync_mutex);
        trajectory_done = true;

        camera_cv.notify_one();
    }

  private:
    void checkBothDone()
    {
        if (lane_ready && object_ready)
        {
            trajectory_done     = false;
            new_frame_available = false;
            trajectory_cv.notify_one();
        }
    }
};

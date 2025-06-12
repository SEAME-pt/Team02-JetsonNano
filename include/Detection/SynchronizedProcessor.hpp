#pragma once

#include <condition_variable>
#include <queue>             
#include <memory>            
#include "opencv2/opencv.hpp"
// #include <opencv2/cudawarping.hpp>
// #include <opencv2/cudaimgproc.hpp>
// #include <opencv2/core/cuda.hpp>

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
    SynchronizedProcessor();
    ~SynchronizedProcessor();
    
    void setNewFrame(const cv::Mat& frame);

    cv::Mat getLaneFrame();

    cv::Mat getObjectFrame();

    void laneDone(const cv::Mat& result);

    // Object thread calls this when done
    void objectDone(const cv::Mat& result);

    void getProcessingData(cv::Mat& original, cv::Mat& lane_mask,
                           cv::Mat& object_mask);

    void trajectoryDone();

    void shutdown();

  private:
    void checkBothDone();
};

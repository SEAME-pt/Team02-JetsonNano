#pragma once

#include "opencv2/opencv.hpp"
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include "cuda.h"
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include "LaneDetectorPublisher.hpp"
#include "GPUInference.hpp"

class LaneDetector
{
  private:
    cv::cuda::Stream cv_stream;
    GPUInference* gpuInference;

    const int height_;
    const int width_;

  public:
    LaneDetector(const std::string& enginePath, int height, int width);
    ~LaneDetector();
    void detect(cv::Mat& frame, cv::Mat& result);

  private:
    void preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame);
};
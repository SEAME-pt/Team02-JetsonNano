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

#define WIDTH 256
#define HEIGHT 128

class LaneDetector
{
  private:
    cv::cuda::Stream cv_stream;
  
    GPUInference* gpuInference;

  public:
    LaneDetector(const std::string& enginePath);
    ~LaneDetector();
    void detect(cv::Mat& frame, cv::Mat& result);

  private:
    void preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame);
};
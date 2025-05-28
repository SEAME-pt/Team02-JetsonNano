#pragma once

#include "opencv2/opencv.hpp"
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include "cuda.h"
#include "NvInfer.h"
#include "NvOnnxParser.h"
#include <zenoh.hxx>
#include <omp.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <deque>
#include <cmath>
#include <sys/time.h>

#include "Logger.hpp"
#include "CAN.hpp"
#include "GPUInference.hpp"

#define WIDTH 256
#define HEIGHT 128

class ObjectDetector
{
  private:
    cv::cuda::Stream cv_stream;
    GPUInference* gpuInference;

  public:
    ObjectDetector(const std::string& enginePath);
    ~ObjectDetector();

    void detect(cv::Mat& frame, cv::Mat& result);

  private:
    void preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame);
};
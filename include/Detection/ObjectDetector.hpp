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
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;
    std::optional<zenoh::Publisher> speed_lock_publisher_;

    GPUInference* gpuInference;
    CAN* canBus;

    bool is_emergency_stop = false;


  public:
    ObjectDetector(const std::string& enginePath,
                   std::shared_ptr<zenoh::Session> session);
    ~ObjectDetector();

    void detect(cv::Mat& frame);

  private:
    void preProcess(cv::Mat& frame, cv::Mat& preprocessedFrame);
    void postProcess(cv::Mat& frame, cv::Mat& class_mask);
    void createExecutionContext(const std::string& enginePath);

    bool checkForwardCollision(const cv::Mat& segmentation_mask);

    void publishSpeedLock(const std::string &value_str);
};
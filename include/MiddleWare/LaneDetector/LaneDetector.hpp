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
#include "CAN.hpp"

#define WIDTH 256
#define HEIGHT 128
#define INPUT_SIZE 3
#define OUTPUT_SIZE 1

class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
        {
            std::cout << msg << std::endl;
        }
    }
};

extern Logger logger;

class LaneDetector
{
  private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;

    std::shared_ptr<nvinfer1::IExecutionContext> context;
    cudaEvent_t start;
    cudaEvent_t stop;
    cudaStream_t stream;
    cv::cuda::Stream cv_stream;
    void* inputDevice;
    void* outputDevice;
    float* inputData;
    float* outputData;

    CAN* canBus;

  public:
    LaneDetector(const std::string& enginePath,
                   std::shared_ptr<zenoh::Session> session);
    ~LaneDetector();
    void detect(cv::Mat& frame);

  private:
    void preProcess(const cv::Mat& frame);
    void postProcess(cv::Mat& frame);
    void createExecutionContext(const std::string& enginePath);
};
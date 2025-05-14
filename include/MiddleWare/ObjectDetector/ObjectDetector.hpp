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

#define WIDTH 256
#define HEIGHT 128
#define INPUT_SIZE 3
#define OUTPUT_SIZE 10

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

class ObjectDetector
{
  private:
    std::shared_ptr<nvinfer1::IExecutionContext> context;
    cudaEvent_t start;
    cudaEvent_t stop;
    cudaStream_t stream;
    cv::cuda::Stream cv_stream;
    void* inputDevice;
    void* outputDevice;
    float* inputData;
    float* outputData;

    cv::VideoCapture cap;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat map1, map2;

    const int FRAME_SKIP;
    int frame_count;

  public:
    ObjectDetector(const std::string& enginePath, const std::string& pipeline);
    ~ObjectDetector();
    void setCalibrationParameters(void);

    void detect(cv::Mat& frame);
    void run();

  private:
    void preProcess(const cv::Mat& frame);
    void postProcess(cv::Mat& frame);
    void createExecutionContext(const std::string& enginePath);

    double getCurrentTime();
};  
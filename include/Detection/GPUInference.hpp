#pragma once

#include "opencv2/opencv.hpp"
// #include <opencv2/cudawarping.hpp>
// #include <opencv2/cudaimgproc.hpp>
// #include <opencv2/core/cuda.hpp>
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
#include "Logger.hpp"
#include "IPM.hpp"
#include "LaneDetectorPublisher.hpp"
#include "KalmanFilter.hpp"

class GPUInference
{
  private:
    std::shared_ptr<nvinfer1::IExecutionContext> context;
    std::string enginePath_;
    cudaEvent_t start;
    cudaEvent_t stop;
    cudaStream_t stream;
    void* inputDevice;
    void* outputDevice;
    float* inputData;
    float* outputData;
    int inputChannels_;
    int outputChannels_;
    int height_;
    int width_;

  public:
    GPUInference(const std::string &enginePath, int inputChannels, int outputChannels, int height, int width);
    ~GPUInference();

    void init();
    void inference();
    void copyToGPU(cv::Mat& preprocessedFrame);
    void copyToCPUBinaryOutput(cv::Mat& outputMask);
    void copyToCPUClassOutput(cv::Mat& outputMask);
    int copyToCPUTrafficOutput();

  private:
    void createExecutionContext(const std::string& enginePath);
};
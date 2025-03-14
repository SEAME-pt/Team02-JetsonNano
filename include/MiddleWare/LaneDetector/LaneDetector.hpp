#include "opencv2/opencv.hpp"
#include "cuda.h"
#include "NvInfer.h"
#include "NvOnnxParser.h"
#include <omp.h>
#include <fstream>
#include <iostream>
#include <memory>

class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
        {
            std::cout << msg << std::endl;
        }
    }
} logger;

class LaneDetector
{
  private:
    std::shared_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t stream;
    void* inputDevice;
    void* outputDevice;
    float* inputData;
    float* outputData;

  public:
    LaneDetector(const std::string& enginePath);
    ~LaneDetector();

    void detect(cv::Mat& frame);
    void run(const std::string& pipeline);

  private:
    void preProcess(const cv::Mat& frame);
    void postProcess(cv::Mat& frame);
    void createExecutionContext(const std::string& enginePath);
};
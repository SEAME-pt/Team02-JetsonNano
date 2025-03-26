#pragma once

#include "opencv2/opencv.hpp"
#include "cuda.h"
#include "NvInfer.h"
#include "NvOnnxParser.h"
#include <zenoh.hxx>
#include <omp.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include "CAN.hpp"
#include <sys/time.h>
#include "PidController.hpp"
#include "LaneDetectorPublisher.hpp"
#include "LaneDetector.hpp"


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
    cv::VideoCapture cap;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat map1, map2;

    std::vector<cv::Point> prevLeftPoints;
    std::vector<cv::Point> prevRightPoints;

    std::shared_ptr<zenoh::Session> session_;
    std::shared_ptr<LaneDetectorPublisher> publisher_;

    const int FRAME_SKIP;
    cv::KalmanFilter leftLaneKF, rightLaneKF;
    bool kfInitialized = false;
    double laneWidthEstimate = 0.0;
    bool firstFrame;
    int frame_count;

  public:
    LaneDetector(const std::string& enginePath, const std::string& pipeline,
      std::shared_ptr<zenoh::Session> session);
    ~LaneDetector();
    void setCalibrationParameters(void);

    void detect(cv::Mat& frame);
    void run();

  private:
    void preProcess(const cv::Mat& frame);
    void postProcess(cv::Mat& frame);
    void createExecutionContext(const std::string& enginePath);

    cv::Mat regionOfInterest(const cv::Mat& img,
    const std::vector<cv::Point>& vertices);
    cv::Mat polyfit(const cv::Mat& y_vals, const cv::Mat& x_vals, int degree);
    double getCurrentTime();
    void createLanes(std::vector<cv::Point> lanes, cv::Mat& frame);
    void drawLanes(cv::Mat& frame, 
      const std::vector<cv::Point>& leftCurve, 
      const std::vector<cv::Point>& rightCurve);
    void clusterLanePoints(const std::vector<cv::Point>& points, 
      std::vector<cv::Point>& leftPoints,
      std::vector<cv::Point>& rightPoints);
      std::vector<cv::Point> fitCurveToPoints(const std::vector<cv::Point>& points);
    void reassignPointsUsingPreviousFrame(std::vector<cv::Point>& leftPoints, 
      std::vector<cv::Point>& rightPoints);
    void initKalmanFilters(const std::vector<cv::Point>& leftCurve, 
      const std::vector<cv::Point>& rightCurve);
};  
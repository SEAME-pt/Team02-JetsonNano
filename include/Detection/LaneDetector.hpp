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
#include "Logger.hpp"
#include "IPM.hpp"
#include "LaneDetectorPublisher.hpp"
#include "KalmanFilter.hpp"

#define WIDTH 256
#define HEIGHT 128

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
    
    IPM ipm;
    cv::Size bevSize;

    std::vector<cv::Point> prevLeftCurve;
    std::vector<cv::Point> prevRightCurve;

    int leftLaneLastUpdatedFrame = 0;
    int rightLaneLastUpdatedFrame = 0;
    int currentFrame = 0;
    const int MAX_LANE_MEMORY_FRAMES = 20;

    KalmanFilter kalmanFilter;
  public:
    std::shared_ptr<LaneDetectorPublisher> publisher_;

  public:
    LaneDetector(const std::string& enginePath,
                   std::shared_ptr<zenoh::Session> session);
    ~LaneDetector();
    void detect(cv::Mat& frame);

  private:
    void preProcess(const cv::Mat& frame);
    void postProcess(cv::Mat& frame);

    void createLanes(cv::Mat& binary_mask, cv::Mat& frame);
    std::vector<std::vector<cv::Point>> clusterLaneMask(const cv::Mat& laneMask, int kernelSize, int minArea, int maxLanes);
    void mergeLaneComponents(std::vector<std::vector<cv::Point>>& lanePolylines, float maxHorizontalDist, float maxVerticalGap, int width);
    void drawPolyLanes(std::vector<std::vector<cv::Point>> lanePolylines, cv::Mat& allPolylinesViz);

    float calculateLaneDistance(const std::vector<cv::Point>& lane1, const std::vector<cv::Point>& lane2);
    bool validateLaneSeparation(const std::vector<std::vector<cv::Point>>& lanePolylines, float minLaneWidth);
    void isValidMiddleCurve(std::vector<cv::Point>& midCurve, const std::vector<cv::Point>& realLane, cv::Mat& allPolylinesViz, bool isLeftLane, int width);
    
    void createExecutionContext(const std::string& enginePath);
};
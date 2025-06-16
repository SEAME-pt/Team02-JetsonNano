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
#include <sys/stat.h>

#include "CAN.hpp"
#include "Logger.hpp"
#include "IPM.hpp"
#include "LaneDetectorPublisher.hpp"
#include "KalmanFilter.hpp"
#include "GPUInference.hpp"

class TrajectoryDefinition
{
  private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;

    cv::cuda::Stream cv_stream;
  
    CAN* canBus;
    KalmanFilter* kalmanFilter;
    IPM* ipm;
  
    std::vector<cv::Point> prevLeftCurve;
    std::vector<cv::Point> prevRightCurve;

    cv::Mat allPolylinesViz_;
    int frameWidth_;
    int frameHeight_;

    std::deque<float> recentWidths;
    const int MAX_WIDTH_HISTORY = 15;

    int leftLaneLastUpdatedFrame = 0;
    int rightLaneLastUpdatedFrame = 0;
    int currentFrame = 0;
    const int MAX_LANE_MEMORY_FRAMES = 15;

    std::optional<zenoh::Publisher> coeffs_publisher_;
  
    std::optional<zenoh::Publisher> speed_lock_publisher_;
    bool is_emergency_stop = false;

    const int height_;
    const int width_;
    
  public:
    std::optional<zenoh::Publisher> ipm_frame_publisher_;
    std::optional<zenoh::Publisher> frame_publisher_;
    std::optional<zenoh::Publisher> lane_mask_publisher_;
    std::optional<zenoh::Publisher> class_mask_publisher_;
    std::shared_ptr<LaneDetectorPublisher> publisher_;

  public:
    TrajectoryDefinition(std::shared_ptr<zenoh::Session> session, const int height, const int width);
    ~TrajectoryDefinition();

    void initLocalEnv();
    void initCarlaEnv();

    cv::Mat process(cv::Mat& frame, cv::Mat& binary_mask, cv::Mat& class_mask);
    void publishIPMFrame(const std::string& value_str);
    void publishOrigFrame(const std::string& value_str);
    void publishBinMask(const std::string& value_str);
    void publishClassMask(const std::string& value_str);
  private:
    void createLanes(cv::Mat& frame, cv::Mat& binary_mask, cv::Mat& class_mask);
    std::vector<std::vector<cv::Point>> clusterLaneMask(const cv::Mat& laneMask, int kernelSize, int minArea, int maxLanes);
    void mergeLaneComponents(std::vector<std::vector<cv::Point>>& lanePolylines, float maxHorizontalDist, float maxVerticalGap);
    void onePolyline(std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);
    void twoPolylines(std::vector<std::vector<cv::Point>> lanePolylines, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);
    void drawPolyLanes(std::vector<std::vector<cv::Point>> lanePolylines);
    bool checkIfLeftLane(const std::vector<cv::Point>& lanePolyline);

    float calculateHistoricalLaneWidth();
    void updateLaneWidthHistory(const std::vector<cv::Point>& leftLane, 
                                                const std::vector<cv::Point>& rightLane);

    float calculateLaneDistance(const std::vector<cv::Point>& lane1, const std::vector<cv::Point>& lane2);
    bool validateLaneSeparation(const std::vector<std::vector<cv::Point>>& lanePolylines, float minLaneWidth);
    void checkPredicedCurve(std::vector<cv::Point>& predictedCurve, const std::vector<cv::Point>& realLane, bool isLeftLane);
    void defineTrajectoryCurve(std::vector<cv::Point>& midCurve, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);
    void drawCurves(std::vector<cv::Point>& midCurve, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);
    void createMidPointError(std::vector<cv::Point>& midCurve);  
    void defineLanePolyline(std::vector<cv::Point>& curve);
    
    void checkForwardCollision(const cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve);
    void publishSpeedLock(const std::string &value_str);
    void publishCoeffs(const std::string &value_str);
};
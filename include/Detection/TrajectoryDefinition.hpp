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
#include <sys/stat.h>

#include "CAN.hpp"
#include "Logger.hpp"
#include "IPM.hpp"
#include "LaneDetectorPublisher.hpp"
#include "KalmanFilter.hpp"
#include "GPUInference.hpp"
#include "ObstacleAvoidance.hpp"
#include "AdaptiveCruiseControl.hpp"


class TrajectoryDefinition
{
  private:
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::PosixShmProvider> provider_;

    // cv::cuda::Stream cv_stream;
  
    KalmanFilter* kalmanFilter;
    IPM* ipm;
    ObstacleAvoidance* avoidance;
    AdaptiveCruiseControl* accontroller;

  
    std::vector<cv::Point> prevLeftCurve;
    std::vector<cv::Point> prevRightCurve;

    cv::Mat allPolylinesViz_;
    int frameWidth_;
    int frameHeight_;

    std::deque<float> recentWidths;
    const int MAX_WIDTH_HISTORY = 15;

    const float distance_percentage = 0.70;

    int leftLaneLastUpdatedFrame = 0;
    int rightLaneLastUpdatedFrame = 0;
    int currentFrame = 0;
    const int MAX_LANE_MEMORY_FRAMES = 15;

    std::optional<zenoh::Publisher> coeffs_publisher_;
  
    std::optional<zenoh::Publisher> emergency_brake_publisher_;
    bool is_emergency_stop = false;

    const int height_;
    const int width_;

    // IPM properties
    float nearDistance_;
    float farDistance_;
    float laneWidth_;


    std::vector<cv::Point> mpcPoints_;

    int distanceToObstacle_;

    
  public:
    std::optional<zenoh::Publisher> ipm_frame_publisher_;
    std::optional<zenoh::Publisher> frame_publisher_;
    std::optional<zenoh::Publisher> lane_mask_publisher_;
    std::optional<zenoh::Publisher> class_mask_publisher_;
    std::optional<zenoh::Publisher> lkas_publisher_;
    std::optional<zenoh::Publisher> sae_2_disable_publisher_;
    std::optional<zenoh::Publisher> autonomy_env_enable_;
    std::optional<zenoh::Publisher> acc_speed_publisher_;

    std::optional<zenoh::Subscriber<void>> mpc_trajectory_subscriber;
    std::optional<zenoh::Subscriber<void>> activeAutonomyLevel_subscriber_;
    
    std::string activeAutonomyLevel_;

    bool sae_3Enable_ = false;
    bool sae_4Enable_ = false;
    
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
    void publishEmergencyBrake(const std::string &value_str);
    void publishCoeffs(std::vector<cv::Point>& curve);
    void publishLKAS(const std::string &value_str);
    void publishSAE2Disable(const std::string& value_str);
    void publishAutonomyEnvEnable(const std::string& value_str);
    void publishACC(const std::string& value_str);

  private:

    void createLanes(cv::Mat& frame, cv::Mat& binary_mask, cv::Mat& class_mask);
    std::vector<std::vector<cv::Point>> clusterLaneMask(const cv::Mat& laneMask, int kernelSize, int minArea, int maxLanes);
    void clusterObjMask(const cv::Mat& classMask, int kernelSize);
    void defineLaneEnv(std::vector<std::vector<cv::Point>> &lanePolylines, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve, std::vector<cv::Mat> &coeffsSave);
    void onePolyline(std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);
    void twoPolylines(std::vector<std::vector<cv::Point>> lanePolylines, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);
    void drawPolyLanes(std::vector<std::vector<cv::Point>> lanePolylines);
    bool checkIfLeftLane(const std::vector<cv::Point>& lanePolyline);

    void filterFalseLanes(std::vector<std::vector<cv::Point>> &lanePolylines, std::vector<cv::Mat> &coeffsSave);
    void lowerPointLaneDefinition(std::vector<std::vector<cv::Point>> &lanePolylines, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);

    float calculateHistoricalLaneWidth();
    void updateLaneWidthHistory(const std::vector<cv::Point>& leftLane, 
                                                const std::vector<cv::Point>& rightLane);

    float calculateLaneDistance(const std::vector<cv::Point>& lane1, const std::vector<cv::Point>& lane2);
    bool validateLaneSeparation(const std::vector<std::vector<cv::Point>>& lanePolylines, float minLaneWidth);
    void predictCurve(std::vector<cv::Point>& predictedCurve, const std::vector<cv::Point>& realLane, bool isLeftLane);
    void defineTrajectoryCurve(std::vector<cv::Point>& midCurve, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);
    void drawCurves(std::vector<cv::Point>& midCurve, std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve);
    void createMidPointError(std::vector<cv::Point>& midCurve);  
    cv::Mat defineLanePolyline(std::vector<cv::Point>& curve);
    void checkLKASEnvEnable(std::vector<cv::Point>& leftCurve, std::vector<cv::Point>& rightCurve, cv::Mat& leftCoeffs, cv::Mat& rightCoeffs);
    void checkAutonomyEnvEnable(std::vector<cv::Point>& midCurve);
    bool isCurveStraight(const cv::Mat& coeffs, double threshold);
    
    bool checkForwardCollision(const cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve);

    void obstacleAvoidance(cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve);
    cv::Point findClosestPointAtY(const std::vector<cv::Point>& curve, int targetY);


    void mpcDebug(void);
    void setAutonomousDriveState(std::string activeAutonomyLevel);
    void adaptiveSpeedControl(cv::Mat& segmentation_mask, std::vector<cv::Point>& midCurve);

    cv::Point2f calculateSmoothTangent(const std::vector<cv::Point>& curve, size_t index);
    std::vector<cv::Point> smoothPolygon(const std::vector<cv::Point>& polygon);
};  
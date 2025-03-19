#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <zenoh.hxx>
#include "PidController.hpp"
#include "LaneDetectorPublisher.hpp"
#include <opencv2/video/tracking.hpp>

class LaneDetectorCV
{
private:
    // Video capture
    cv::VideoCapture cap;
    
    std::vector<cv::Point> prevLeftCurve;
    std::vector<cv::Point> prevRightCurve;
    std::vector<cv::Point> prevMidCurve;
    
    // Zenoh communication
    std::shared_ptr<zenoh::Session> session_;
    std::shared_ptr<LaneDetectorPublisher> publisher_;

    
    // Lane tracking state
    cv::Vec4i prevLeftLine, prevRightLine, prevMidLine;
    double laneWidthEstimate;
    bool firstFrame;
    int frame_count;
    
    // Processing parameters
    const int FRAME_SKIP;

    //Kalman filter methods to stabilize lanes
    cv::KalmanFilter leftLaneKF;
    cv::KalmanFilter rightLaneKF;
    bool kfInitialized;
    
public:
    LaneDetectorCV(const std::string& pipeline, std::shared_ptr<zenoh::Session> session);
    ~LaneDetectorCV();
    
    void run();
    void detect(cv::Mat& frame);

private:
    cv::Mat regionOfInterest(const cv::Mat &img, const std::vector<cv::Point>& vertices);
    double getCurrentTime();
    cv::Vec4i extrapolateLine(const std::vector<cv::Vec4i>& laneLines);
    cv::Mat polyfit(const cv::Mat& y_vals, const cv::Mat& x_vals, int degree) ;
    std::vector<cv::Point> extrapolatePolynomialCurve(const std::vector<cv::Vec4i>& laneLines);
    void initKalmanFilters(const std::vector<cv::Point>& leftCurve, const std::vector<cv::Point>& rightCurve);

};
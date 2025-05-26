#pragma once

#include "opencv2/opencv.hpp"
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda.hpp>

class KalmanFilter
{
  private:
    cv::KalmanFilter rightLaneKF;
    cv::KalmanFilter leftLaneKF;

  public:
    KalmanFilter(const std::string& enginePath,
                   std::shared_ptr<zenoh::Session> session);
    ~KalmanFilter();

  private:
    void initializeKalmanFilters();

    std::vector<cv::Point> reconstructLaneFromCoefficients(const cv::Mat& coeffs, int height);
    cv::Mat extractPolynomialCoefficients(const std::vector<cv::Point>& laneCurve);
    Mat polyfit(const Mat& y_vals, const Mat& x_vals, int degree);

    void updateLeftLaneFilter(const std::vector<cv::Point>& lane);
    void updateRightLaneFilter(const std::vector<cv::Point>& lane);
    std::vector<cv::Point> KalmanFilter::predictLeftLaneCurve();
    std::vector<cv::Point> KalmanFilter::predictRightLaneCurve();
};
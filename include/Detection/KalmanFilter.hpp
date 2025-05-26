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
    KalmanFilter();
    ~KalmanFilter();

    void updateLeftLaneFilter(const std::vector<cv::Point>& lane);
    void updateRightLaneFilter(const std::vector<cv::Point>& lane);
    std::vector<cv::Point> predictLeftLaneCurve(int height, int width);
    std::vector<cv::Point> predictRightLaneCurve(int height, int width);

  private:
    void initializePolynomialKalmanFilters();
  
    std::vector<cv::Point> reconstructLaneFromCoefficients(const cv::Mat& coeffs, int height, int width);
    cv::Mat extractPolynomialCoefficients(const std::vector<cv::Point>& laneCurve);
    cv::Mat polyfit(const cv::Mat& y_vals, const cv::Mat& x_vals, int degree);

};
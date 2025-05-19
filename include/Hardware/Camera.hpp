#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

class Camera {
private:
    cv::VideoCapture cap;
    const int FRAME_SKIP;
    int frame_count;
    cv::Mat currentFrame;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat map1, map2;
    
    std::thread captureThread;
    std::atomic<bool> running{false};
    std::mutex frameMutex;
    
    void captureLoop();

public:
    Camera(const std::string& pipeline, const std::string& calibrationFile = "calibration.yml");
    ~Camera();

    void startCapture();
    void stopCapture();
    
    cv::Mat getFrame();
    
    void setCalibrationParameters(const std::string& calibrationFile);
};
#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <zenoh.hxx>

class Camera {
private:
    cv::VideoCapture cap;

    cv::Mat currentFrame;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat map1, map2;
    
    std::thread captureThread;
    std::atomic<bool> running{false};
    std::mutex frameMutex;

    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::Subscriber<void>> carla_frame;

    bool useZenohSubscription = true;
    
    void captureLoop();

public:
    Camera(std::shared_ptr<zenoh::Session> session);
    ~Camera();

    void initLocalEnv(const std::string& pipeline, const std::string& calibrationFile);
    void initCarlaEnv();
    void initLVideoTestEnv(const std::string& video, const std::string& calibrationFile);

    void startCapture();
    void stopCapture();
    
    cv::Mat getFrame();
    
    void setCalibrationParameters(const std::string& calibrationFile);
};
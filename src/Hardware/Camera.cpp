#include "Camera.hpp"
#include <iostream>

using namespace cv;
using namespace std;

Camera::Camera(const std::string& pipeline, const std::string& calibrationFile) : FRAME_SKIP(3), frame_count(0)
{
    cap.open(pipeline, cv::CAP_GSTREAMER)

    if (!cap.isOpened())
    {
        throw std::runtime_error("Error opening video stream");
    }
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cap >> currentFrame;
    if (currentFrame.empty()) {
        cv::destroyAllWindows();
        throw std::runtime_error("Error reading initial frame");
    }

    this->setCalibrationParameters(calibrationFile);

    Size imageSize = currentFrame.size();
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            cameraMatrix, imageSize, CV_16SC2, map1, map2);

}

Camera::~Camera()
{
    stopCapture();
}

void Camera::setCalibrationParameters(const std::string& calibrationFile)
{
    FileStorage fs(calibrationFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Failed to open calibration.yml" << endl;
        return;
    }
    Mat tempMatrix, tempCoeffs;
    fs["CameraMatrix"] >> tempMatrix;
    fs["DistCoeffs"] >> tempCoeffs;
    fs.release();
    this->cameraMatrix = tempMatrix;
    this->distCoeffs   = tempCoeffs;
}

void Camera::startCapture()
{
    if (!running) {
        running = true;
        captureThread = std::thread(&Camera::captureLoop, this);
    }
}

void Camera::stopCapture()
{
    running = false;
    if (captureThread.joinable()) {
        captureThread.join();
    }
}

void Camera::run()
{
    while (running)
    {
        Mat frame;
        cap >> frame;

        if (frame.empty()) {
            std::cerr << "Empty frame encountered" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (frame_count % FRAME_SKIP == 0)
        {
            cv::Mat undistorted;
            remap(frame, undistorted, map1, map2, INTER_LINEAR);

            std::lock_guard<std::mutex> lock(frameMutex);
            currentFrame = undistorted.clone();
        }
        frame_count++;
    }
}

cv::Mat Camera::getFrame()
{
    std::lock_guard<std::mutex> lock(frameMutex);
    return currentFrame.clone();
}

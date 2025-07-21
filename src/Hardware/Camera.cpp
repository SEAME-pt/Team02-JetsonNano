#include "Camera.hpp"
#include <iostream>
#include <future>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

Camera::Camera(std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
}

Camera::~Camera()
{
    stopCapture();
}

void Camera::initLocalEnv(const std::string& pipeline,
                          const std::string& calibrationFile)
{
    useZenohSubscription = false;

    cap.open(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        std::cerr << "Failed to open camera with pipeline: " << pipeline
                  << std::endl;
        throw std::runtime_error("Failed to open camera");
    }

    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cap >> currentFrame;
    if (currentFrame.empty())
    {
        cv::destroyAllWindows();
        throw std::runtime_error("Error reading initial frame");
    }

    this->setCalibrationParameters(calibrationFile);

    Size imageSize = currentFrame.size();
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix,
                            imageSize, CV_16SC2, map1, map2);
}

void Camera::initLVideoTestEnv(const std::string& video)
{
    useZenohSubscription = false;

    cap.open(video);

    if (!cap.isOpened())
    {
        std::cerr << "Failed to open video: " << video << std::endl;
        throw std::runtime_error("Failed to open camera");
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cap.set(cv::CAP_PROP_FPS, 1);

    cap >> currentFrame;
    if (currentFrame.empty())
    {
        cv::destroyAllWindows();
        throw std::runtime_error("Error reading initial frame");
    }
}

void Camera::initCarlaEnv()
{
    useZenohSubscription = true;

    {
        std::lock_guard<std::mutex> lock(frameMutex);
        currentFrame = cv::Mat();
    }

    carla_frame.emplace(session_->declare_subscriber(
        "carla/frame",
        [this](const zenoh::Sample& sample)
        {
            try
            {
                std::vector<uint8_t> data = sample.get_payload().as_vector();
                cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);

                if (!img.empty())
                {
                    std::lock_guard<std::mutex> lock(frameMutex);
                    currentFrame = img.clone();
                }
                else
                {
                    std::cerr << "Failed to decode image" << std::endl;
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error processing image data: " << e.what()
                          << std::endl;
            }
        },
        zenoh::closures::none));
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
    if (!running)
    {
        running       = true;
        captureThread = std::thread(&Camera::captureLoop, this);
    }
}

void Camera::stopCapture()
{
    running = false;
    if (captureThread.joinable())
    {
        captureThread.join();
    }
}

void Camera::captureLoop()
{
    while (running)
    {
        Mat frame;

        if (!useZenohSubscription)
        {
            cap >> frame;

            if (frame.empty())
            {
                std::cerr << "Empty frame encountered" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            cv::Mat undistorted;
            remap(frame, undistorted, map1, map2, INTER_LINEAR);

            std::lock_guard<std::mutex> lock(frameMutex);
            currentFrame = undistorted.clone();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

cv::Mat Camera::getFrame()
{
    std::lock_guard<std::mutex> lock(frameMutex);
    return currentFrame.clone();
}

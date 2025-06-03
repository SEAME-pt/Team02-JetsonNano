#include "Camera.hpp"
#include <iostream>
#include <future>
#include <chrono>
#include <thread> 

using namespace cv;
using namespace std;

static std::vector<uint8_t> base64_decode(const std::string& base64_text) {
    static const std::string base64_chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    
    std::vector<uint8_t> decoded;
    int i = 0;
    int j = 0;
    uint8_t char_array_4[4], char_array_3[3];
    
    for (size_t idx = 0; idx < base64_text.size(); ++idx) {
        char c = base64_text[idx];
        if (c == '=' || std::string::npos == base64_chars.find(c)) 
            continue;
            
        char_array_4[i++] = c;
        if (i == 4) {
            for (i = 0; i < 4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);
                
            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
            
            for (i = 0; i < 3; i++)
                decoded.push_back(char_array_3[i]);
            i = 0;
        }
    }
    
    if (i) {
        for (j = i; j < 4; j++)
            char_array_4[j] = 0;
            
        for (j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);
            
        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
        
        for (j = 0; j < i - 1; j++)
            decoded.push_back(char_array_3[j]);
    }
    
    return decoded;
}

Camera::Camera(const std::string& pipeline, const std::string& calibrationFile, std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    useZenohSubscription = false;

    try {
        std::future<bool> future = std::async(std::launch::async, [&]() {
            return cap.open(pipeline, cv::CAP_GSTREAMER);
        });
        
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
            std::cerr << "Timeout waiting for camera to open" << std::endl;
            throw std::runtime_error("Camera open timeout");
        }
        
        if (!cap.isOpened()) {
            std::cerr << "Failed to open camera with pipeline: " << pipeline << std::endl;
            throw std::runtime_error("Failed to open camera");
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
    } catch (const std::runtime_error& e) {
        std::cerr << "Camera error: " << e.what() << ", switching to Zenoh subscription" << std::endl;
        useZenohSubscription = true;

        carla_frame.emplace(session_->declare_subscriber(
        "carla/frame",
        [this](const zenoh::Sample& sample)
        {
            std::cout << "sub" << std::endl;
            try {
                // Get payload as string (base64 encoded data)
                std::string base64_str = sample.get_payload().as_string();
                
                // Decode from base64
                std::vector<uint8_t> img_bytes = base64_decode(base64_str);
                
                // Convert to cv::Mat using imdecode
                cv::Mat img = cv::imdecode(img_bytes, cv::IMREAD_COLOR);
                
                if (!img.empty()) {
                    // Update the current frame
                    std::lock_guard<std::mutex> lock(frameMutex);
                    currentFrame = img.clone();
                
                } else {
                    std::cerr << "Failed to decode image" << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error processing image data: " << e.what() << std::endl;
            }
        },
        zenoh::closures::none));
    }
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

        if (!useZenohSubscription) {
            cap >> frame;

            if (frame.empty()) {
                std::cerr << "Empty frame encountered" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            cv::Mat undistorted;
            remap(frame, undistorted, map1, map2, INTER_LINEAR);

            std::lock_guard<std::mutex> lock(frameMutex);
            currentFrame = undistorted.clone();
        }
    
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

cv::Mat Camera::getFrame()
{
    std::lock_guard<std::mutex> lock(frameMutex);
    return currentFrame.clone();
}

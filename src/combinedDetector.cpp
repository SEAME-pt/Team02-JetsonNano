#include "LaneDetector.hpp"
#include "ObjectDetector.hpp"
#include "Camera.hpp"
#include <thread>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

std::atomic<bool> running(true);
Mat displayFrame;
std::mutex displayMutex;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

void laneDetectionThread(LaneDetector* detector, Camera* camera) {
    while (running) {
        cv::Mat frame = camera->getFrame();
        
        if (!frame.empty()) {
            detector->detect(frame);

            std::lock_guard<std::mutex> lock(displayMutex);
            frame.copyTo(displayFrame(Rect(0, 0, displayFrame.cols/2, displayFrame.rows)));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void objectDetectionThread(ObjectDetector* detector, Camera* camera) {
    while (running) {
        cv::Mat frame = camera->getFrame();
        
        if (!frame.empty()) {
            detector->detect(frame);
            
            std::lock_guard<std::mutex> lock(displayMutex);
            frame.copyTo(displayFrame(Rect(displayFrame.cols/2, 0, displayFrame.cols/2, displayFrame.rows)));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void displayThread() {
    cv::namedWindow("Combined Detection", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Combined Detection", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    
    while (running) {
        cv::Mat currentDisplay;
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            if (!displayFrame.empty())
                currentDisplay = displayFrame.clone();
        }
        
        if (!currentDisplay.empty()) {
            cv::line(currentDisplay, 
                     cv::Point(currentDisplay.cols/2, 0),
                     cv::Point(currentDisplay.cols/2, currentDisplay.rows),
                     cv::Scalar(0, 0, 255), 2);
                     
            cv::putText(currentDisplay, "Lane Detection", cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            cv::putText(currentDisplay, "Object Detection", cv::Point(currentDisplay.cols/2 + 10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                        
            cv::imshow("Combined Detection", currentDisplay);
        }
        
        if (cv::waitKey(1) == 'q')
            running = false;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

int main(int argc, char** argv)
{   
    try
    {
        std::shared_ptr<zenoh::Session> session;
        if (argc == 2)
        {
            auto config = Config::from_file(std::string(argv[1]));
            session     = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }
        else
        {
            auto config = Config::create_default();
            session     = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }

        const std::string pipeline =
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=(int)640, height=(int)480, "
            "format=NV12, framerate=(fraction)30/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink";
    
        Camera camera(pipeline, "calibration.yml");
        LaneDetector laneDetector("/home/team02/lane_Mob1_epoch_48.engine", session);
        ObjectDetector objDetector("/home/team02/obj_MOB_1_epoch_133.engine", session);

        camera.startCapture();

        thread laneThread(laneDetectionThread, &laneDetector, &camera);
        thread objThread(objectDetectionThread, &objDetector, &camera);
        thread dispThread(displayThread);

        if (dispThread.joinable()) {
            dispThread.join();
        }

        if (laneThread.joinable()) {
            laneThread.join();
        }
        if (objThread.joinable()) {
            objThread.join();
        }

        camera.stopCapture();
        cv::destroyAllWindows();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

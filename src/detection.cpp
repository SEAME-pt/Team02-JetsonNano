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
    cv::namedWindow("Lane Detection", cv::WINDOW_NORMAL);
    
    while (running) {
        cv::Mat frame = camera->getFrame();
        
        if (!frame.empty()) {
            detector->detect(frame);
            cv::imshow("Lane Detection", frame);
            cv::waitKey(1);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void objectDetectionThread(ObjectDetector* detector, Camera* camera) {
    cv::namedWindow("Object Detection", cv::WINDOW_NORMAL);
    
    while (running) {
        cv::Mat frame = camera->getFrame();
        
        if (!frame.empty()) {
            detector->detect(frame);
            cv::imshow("Object Detection", frame);
            cv::waitKey(1);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

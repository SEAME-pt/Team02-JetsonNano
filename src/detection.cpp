#include "LaneDetector.hpp"
#include "ObjectDetector.hpp"
#include "TrajectoryDefinition.hpp"
#include "Camera.hpp"
#include <thread>
#include <atomic>
#include <condition_variable>  // Add this
#include <queue>              // Add this
#include <memory>             // Add this
#include <iostream> 
#include <csignal> 

using namespace cv;
using namespace std;
using namespace zenoh;

std::atomic<bool> running(true);

class SynchronizedProcessor {
private:
    std::mutex sync_mutex;
    std::condition_variable inference_cv;
    std::condition_variable trajectory_cv;
    std::condition_variable camera_cv;
    
    cv::Mat current_frame;
    cv::Mat lane_binary_mask;
    cv::Mat object_class_mask;
    
    bool lane_ready = true;
    bool object_ready = true;
    bool trajectory_done = true;
    bool new_frame_available = false;
    int frame_id = 0;
    
public:
    void setNewFrame(const cv::Mat& frame) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        camera_cv.wait(lock, [this]() { 
            return lane_ready && object_ready && trajectory_done; 
        });
        
        frame.copyTo(current_frame);
        
        lane_ready = false;
        object_ready = false;
        new_frame_available = true;
        
        inference_cv.notify_all();
    }
    
    // Lane detection thread calls this
    cv::Mat getLaneFrame() {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        inference_cv.wait(lock, [this]() { return new_frame_available && !lane_ready; });
        
        return current_frame.clone();
    }

    cv::Mat getObjectFrame() {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        inference_cv.wait(lock, [this]() { return new_frame_available && !object_ready; });
        
        return current_frame.clone();
    }
    
    void laneDone(const cv::Mat& result) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        result.copyTo(lane_binary_mask);
        lane_ready = true;
        
        checkBothDone();
    }
    
    // Object thread calls this when done
    void objectDone(const cv::Mat& result) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        result.copyTo(object_class_mask);
        object_ready = true;
        
        checkBothDone();
    }
    
    void getProcessingData(cv::Mat& original, cv::Mat& lane_mask, cv::Mat& object_mask) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        trajectory_cv.wait(lock, [this]() { 
            return lane_ready && object_ready && !trajectory_done; 
        });
        
        current_frame.copyTo(original);
        lane_binary_mask.copyTo(lane_mask);
        object_class_mask.copyTo(object_mask);
    }
    
    void trajectoryDone() {
        std::unique_lock<std::mutex> lock(sync_mutex);
        trajectory_done = true;
        
        camera_cv.notify_one();
    }
    
private:
    void checkBothDone() {
        if (lane_ready && object_ready) {
            trajectory_done = false;
            new_frame_available = false;
            trajectory_cv.notify_one();
        }
    }
};

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

void cameraThreadFunction(Camera* camera, SynchronizedProcessor* processor) {
    while (running) {
        cv::Mat frame = camera->getFrame();
        
        if (!frame.empty()) {
            processor->setNewFrame(frame);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void laneDetectionThreadFunction(LaneDetector* detector, SynchronizedProcessor* processor) {
    while (running) {
        cv::Mat frame = processor->getLaneFrame();
        
        cv::Mat result;
        detector->detect(frame, result);
        
        processor->laneDone(result);
    }
}

void objectDetectionThreadFunction(ObjectDetector* detector, SynchronizedProcessor* processor) {
    while (running) {
        cv::Mat frame = processor->getObjectFrame();
        
        cv::Mat result;
        detector->detect(frame, result);
        
        processor->objectDone(result);
    }
}

void trajectoryThreadFunction(TrajectoryDefinition* trajectoryDef, SynchronizedProcessor* processor) {
    cv::namedWindow("Trajectory", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Trajectory", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    
    cv::Mat original_frame, lane_mask, object_mask;
    
    while (running) {
        processor->getProcessingData(original_frame, lane_mask, object_mask);
        
        trajectoryDef->process(original_frame, lane_mask, object_mask);
        
        cv::imshow("Trajectory", original_frame);
        cv::waitKey(1);
        
        processor->trajectoryDone();
    }
    
    cv::destroyWindow("Trajectory");
}

int main(int argc, char** argv)
{   
    signal(SIGINT, signalHandler);

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

        SynchronizedProcessor processor;
    
        Camera camera(pipeline, "calibration.yml");
        LaneDetector laneDetector("/home/team02/Models/engine/lane_Yolo2_epoch_45.engine");
        ObjectDetector objDetector("/home/team02/Models/engine/obj_MOB_1_epoch_133.engine");
        TrajectoryDefinition trajectoryDefinition(session);

        camera.startCapture();

        std::thread camThread(cameraThreadFunction, &camera, &processor);
        std::thread laneThread(laneDetectionThreadFunction, &laneDetector, &processor);
        std::thread objThread(objectDetectionThreadFunction, &objDetector, &processor);
        std::thread trajThread(trajectoryThreadFunction, &trajectoryDefinition, &processor);

        if (camThread.joinable()) {
            camThread.join();
        }
        if (laneThread.joinable()) {
            laneThread.join();
        }
        if (objThread.joinable()) {
            objThread.join();
        }
        if (trajThread.joinable()) {
            trajThread.join();
        }

        camera.stopCapture();

    } catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

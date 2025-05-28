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
    std::shared_ptr<SharedFrameData> frame_data;
    
    bool lane_ready = true;
    bool object_ready = true;
    bool trajectory_done = true;
    bool new_frame_available = false;
    int frame_id = 0;
    
public:
    // Camera thread calls this when both inference threads are ready
    void setNewFrame(const cv::Mat& frame) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        // Wait until both inference threads are ready and trajectory is done
        camera_cv.wait(lock, [this]() { 
            return lane_ready && object_ready && trajectory_done; 
        });
        
        // Set new frame data
        frame.copyTo(current_frame);
        frame_data = std::make_shared<SharedFrameData>();
        frame_data->frame_id = frame_id++;
        frame.copyTo(frame_data->original_frame);
        
        // Reset flags
        lane_ready = false;
        object_ready = false;
        new_frame_available = true;
        
        // Notify inference threads
        inference_cv.notify_all();
    }
    
    // Lane detection thread calls this
    cv::Mat getLaneFrame() {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        // Wait for new frame
        inference_cv.wait(lock, [this]() { return new_frame_available && !lane_ready; });
        
        return current_frame.clone();
    }
    
    // Object detection thread calls this
    cv::Mat getObjectFrame() {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        // Wait for new frame
        inference_cv.wait(lock, [this]() { return new_frame_available && !object_ready; });
        
        return current_frame.clone();
    }
    
    // Lane thread calls this when done
    void laneDone(const cv::Mat& result) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        // Store result
        result.copyTo(frame_data->lane_binary_mask);
        lane_ready = true;
        frame_data->lane_processed = true;
        
        // Check if both are done
        checkBothDone();
    }
    
    // Object thread calls this when done
    void objectDone(const cv::Mat& result) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        // Store result
        result.copyTo(frame_data->object_class_mask);
        object_ready = true;
        frame_data->object_processed = true;
        
        // Check if both are done
        checkBothDone();
    }
    
    // Trajectory thread calls this to get processing data
    std::shared_ptr<SharedFrameData> getProcessingData() {
        std::unique_lock<std::mutex> lock(sync_mutex);
        
        // Wait for both inference results
        trajectory_cv.wait(lock, [this]() { 
            return lane_ready && object_ready && !trajectory_done && frame_data->lane_processed && frame_data->object_processed; 
        });
        
        return frame_data;
    }
    
    // Trajectory thread calls this when done
    void trajectoryDone() {
        std::unique_lock<std::mutex> lock(sync_mutex);
        trajectory_done = true;
        
        // Allow camera to get next frame
        camera_cv.notify_one();
    }
    
private:
    void checkBothDone() {
        if (lane_ready && object_ready) {
            // Both inference threads are done, notify trajectory
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
        // Get latest frame from camera
        cv::Mat frame = camera->getFrame();
        
        if (!frame.empty()) {
            // Pass to synchronized processor
            processor->setNewFrame(frame);
        }
        
        // Small sleep to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void laneDetectionThreadFunction(LaneDetector* detector, SynchronizedProcessor* processor) {
    while (running) {
        // Get frame for processing (blocks until ready)
        cv::Mat frame = processor->getLaneFrame();
        
        // Process frame
        cv::Mat result;
        detector->detect(frame, result);
        
        // Signal completion
        processor->laneDone(result);
    }
}

void objectDetectionThreadFunction(ObjectDetector* detector, SynchronizedProcessor* processor) {
    while (running) {
        // Get frame for processing (blocks until ready)
        cv::Mat frame = processor->getObjectFrame();
        
        // Process frame
        cv::Mat result;
        detector->detect(frame, result);
        
        // Signal completion
        processor->objectDone(result);
    }
}

void trajectoryThreadFunction(TrajectoryDefinition* trajectoryDef, SynchronizedProcessor* processor) {
    cv::namedWindow("Trajectory", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Trajectory", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    
    while (running) {
        // Get processing data when both inferences are done
        auto data = processor->getProcessingData();
        
        // Create output frame
        cv::Mat output_frame;
        data->original_frame.copyTo(output_frame);
        
        // Process trajectory
        trajectoryDef->process(output_frame, data->lane_binary_mask, data->object_class_mask);
        
        // Display result
        cv::imshow("Trajectory", output_frame);
        cv::waitKey(1);
        
        // Signal completion
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
        LaneDetector laneDetector("/home/team02/Models/engine/lane_Yolo2_epoch_45.engine", session);
        ObjectDetector objDetector("/home/team02/Models/engine/obj_MOB_1_epoch_133.engine", session);
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
        if (trajecThread.joinable()) {
            trajecThread.join();
        }

        camera.stopCapture();
        
    } catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

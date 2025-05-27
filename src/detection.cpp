#include "LaneDetector.hpp"
#include "ObjectDetector.hpp"
#include "TrajectoryDefinition.hpp"
#include "Camera.hpp"
#include <thread>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

std::atomic<bool> running(true);

struct SharedFrameData {
    int frame_id;
    cv::Mat original_frame;
    cv::Mat lane_binary_mask;
    cv::Mat object_class_mask;
    std::mutex mutex;
    std::atomic<bool> lane_processed{false};
    std::atomic<bool> object_processed{false};
    std::atomic<bool> trajectory_processed{false};
};

// Frame queue management
class FrameQueue {
private:
    std::queue<std::shared_ptr<SharedFrameData>> frames;
    std::mutex queue_mutex;
    std::condition_variable data_condition;
    int next_frame_id = 0;
    size_t max_size = 1; // Only keep 2 frames at most
    
public:
    void push(const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(queue_mutex);
        
        // If queue is full, drop oldest frame
        if (frames.size() >= max_size) {
            frames.pop(); // Remove oldest frame
            std::cout << "Warning: Dropping frame due to processing lag" << std::endl;
        }
        
        // Add new frame
        auto frame_data = std::make_shared<SharedFrameData>();
        frame_data->frame_id = next_frame_id++;
        frame.copyTo(frame_data->original_frame);
        frames.push(frame_data);
        data_condition.notify_all();
    }
    
    std::shared_ptr<SharedFrameData> pop() {
        std::unique_lock<std::mutex> lock(queue_mutex);
        data_condition.wait(lock, [this] { return !frames.empty(); });
        auto frame_data = frames.front();
        frames.pop();
        return frame_data;
    }
};

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

void cameraThread(Camera* camera, FrameQueue *input_queue) {
    while (running) {
        cv::Mat frame = camera->getFrame();
        
        if (!frame.empty()) {
            input_queue->push(frame);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void laneDetectionThread(LaneDetector* detector, Camera* camera, FrameQueue *input_queue) {
    while (running) {
        std::shared_ptr<SharedFrameData> frame_data = input_queue->pop();

        cv::Mat lane_frame;
        frame_data->original_frame.copyTo(lane_frame);
        
        detector->detect(lane_frame);

        {
            std::lock_guard<std::mutex> lock(frame_data->mutex);
            lane_frame.copyTo(frame_data->lane_binary_mask);
            frame_data->lane_processed = true;
        }
        
        // Add to processed queue if both detectors have finished
        if (frame_data->object_processed) {
            std::lock_guard<std::mutex> lock(processed_mutex);
            processed_queue.push(frame_data);
            processed_condition.notify_one();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void objectDetectionThread(ObjectDetector* detector, Camera* camera, FrameQueue *input_queue) {
    while (running) {
        std::shared_ptr<SharedFrameData> frame_data = input_queue->pop();

        cv::Mat obj_frame;
        frame_data->original_frame.copyTo(obj_frame);
        
        detector->detect(obj_frame);

        {
            std::lock_guard<std::mutex> lock(frame_data->mutex);
            obj_frame.copyTo(frame_data->object_class_mask);
            frame_data->object_processed = true;
        }
        
        // Add to processed queue if both detectors have finished
        if (frame_data->lane_processed) {
            std::lock_guard<std::mutex> lock(processed_mutex);
            processed_queue.push(frame_data);
            processed_condition.notify_one();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void trajectoryDefinitionThread(TrajectoryDefinition* trajectoryDefinition, FrameQueue *input_queue) {
    cv::namedWindow("Trajectory Definition", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Trajectory Definition", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    
    while (running) {
        std::shared_ptr<SharedFrameData> frame_data;
            {
                std::unique_lock<std::mutex> lock(processed_mutex);
                processed_condition.wait(lock, [&] { 
                    return !processed_queue.empty(); 
                });
                frame_data = processed_queue.front();
                processed_queue.pop();
            }
            
            // Process combined results
            cv::Mat output_frame;
            frame_data->original_frame.copyTo(output_frame);
            
            // Now process with trajectory definition, which uses both masks
            trajectoryDefinition.process(output_frame, 
                                       frame_data->lane_binary_mask, 
                                       frame_data->object_class_mask);
        // Display result
        cv::imshow("Trajectory Definition", output_frame);
        cv::waitKey(1);
        
        frame_data->trajectory_processed = true;

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

        FrameQueue input_queue;
        std::queue<std::shared_ptr<SharedFrameData>> processed_queue;
        std::mutex processed_mutex;
        std::condition_variable processed_condition;
    
        Camera camera(pipeline, "calibration.yml");
        LaneDetector laneDetector("/home/team02/Models/engine/lane_Yolo2_epoch_45.engine", session);
        ObjectDetector objDetector("/home/team02/Models/engine/obj_MOB_1_epoch_133.engine", session);
        TrajectoryDefinition trajectoryDefinition(session);

        camera.startCapture();

        thread cameraThread(cameraThread, &camera, &input_queue);
        thread laneThread(laneDetectionThread, &laneDetector, &camera, &input_queue);
        thread objThread(objectDetectionThread, &objDetector, &camera, &input_queue);
        thread trajecThread(trajectoryDefinitionThread, &trajectoryDefinition, &camera, &input_queue);

        if (cameraThread.joinable()) {
            cameraThread.join();
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
        
        cv::destroyAllWindows();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

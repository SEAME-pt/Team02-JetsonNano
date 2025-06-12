#include "LaneDetector.hpp"
#include "ObjectDetector.hpp"
#include "TrajectoryDefinition.hpp"
#include "Camera.hpp"
#include "SynchronizedProcessor.hpp"
#include "utils.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#include <csignal>

using namespace cv;
using namespace std;
using namespace zenoh;

std::atomic<bool> running(true);

void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

void cameraThreadFunction(Camera* camera, SynchronizedProcessor* processor)
{
    while (running)
    {
        cv::Mat frame = camera->getFrame();

        if (!frame.empty())
        {
            processor->setNewFrame(frame);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void laneDetectionThreadFunction(LaneDetector* detector,
                                 SynchronizedProcessor* processor)
{
    while (running)
    {
        cv::Mat frame = processor->getLaneFrame();

        cv::Mat result;
        detector->detect(frame, result);

        processor->laneDone(result);
    }
}

void objectDetectionThreadFunction(ObjectDetector* detector,
                                   SynchronizedProcessor* processor)
{
    while (running)
    {
        cv::Mat frame = processor->getObjectFrame();

        cv::Mat result;
        detector->detect(frame, result);

        processor->objectDone(result);
    }
}

void trajectoryThreadFunction(TrajectoryDefinition* trajectoryDef,
                              SynchronizedProcessor* processor)
{   
    cv::Mat original_frame, lane_mask, object_mask;

    while (running)
    {
        processor->getProcessingData(original_frame, lane_mask, object_mask);

        cv::Mat new_frame = trajectoryDef->process(original_frame, lane_mask, object_mask);

        std::vector<uchar> buffer_ipm_frame;
        std::vector<int> params_ipm = {cv::IMWRITE_JPEG_QUALITY, 20};
        cv::imencode(".jpg", new_frame, buffer_ipm_frame, params_ipm);
        
        trajectoryDef->ipm_frame_publisher_->put(buffer_ipm_frame);

        std::vector<uchar> buffer_original_frame;
        std::vector<int> params_org_frame = {cv::IMWRITE_JPEG_QUALITY, 20};
        cv::imencode(".jpg", original_frame, buffer_original_frame, params_org_frame);
        
        trajectoryDef->frame_publisher_->put(buffer_original_frame);

        std::vector<uchar> buffer_lane_mask;
        std::vector<int> params_lane = {cv::IMWRITE_JPEG_QUALITY, 20};
        cv::imencode(".jpg", lane_mask, buffer_lane_mask, params_lane);
        
        trajectoryDef->lane_mask_publisher_->put(buffer_lane_mask);

        std::vector<uchar> buffer_obj_mask;
        std::vector<int> params_obj = {cv::IMWRITE_JPEG_QUALITY, 20};
        cv::imencode(".jpg", object_mask, buffer_obj_mask, params_obj);
        
        trajectoryDef->class_mask_publisher_->put(buffer_obj_mask);

        processor->trajectoryDone();
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    std::thread camThread, laneThread, objThread, trajThread;

    const int height = 128;
    const int width = 256;

    try
    {
        std::string configFile;
        std::string mode;
        std::string laneDetectionFile;
        std::string objDetectionFile;
        
        if (!parseParameters(argc, argv, configFile, mode)) {
            return -1;
        }

        std::shared_ptr<zenoh::Session> session;
        if (!configFile.empty()) {
            std::cout << "Using configuration from file: " << configFile << std::endl;
            auto config = Config::from_file(configFile);
            session = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }
        else {
            std::cout << "Using default configuration" << std::endl;
            auto config = Config::create_default();
            config.insert_json5("listen/endpoints", "[\"udp/100.73.255.97:7450\"]");
            config.insert_json5("connect/endpoints", "[\"udp/100.73.255.97:7447\"]");
            // config.insert_json5("listen/endpoints", "[\"udp/100.117.122.95:7450\"]");
            // config.insert_json5("connect/endpoints", "[\"udp/100.117.122.95:7447\"]");
            // config.insert_json5("listen/endpoints", "[\"udp/100.119.72.83:7450\"]");
            // config.insert_json5("connect/endpoints", "[\"udp/100.119.72.83:7447\"]");
            session = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }
        std::cout << "Zenoh session created successfully." << std::endl;
        SynchronizedProcessor processor;
        std::cout << "SynchronizedProcessor initialized." << std::endl;
        Camera camera(session);
        std::cout << "Camera initialized." << std::endl;
        TrajectoryDefinition trajectoryDefinition(session, height, width);
        std::cout << "TrajectoryDefinition initialized." << std::endl;

        if (mode == "local") {
            std::cout << "Running in LOCAL mode with physical camera" << std::endl;
            const std::string pipeline =
                "nvarguscamerasrc sensor-id=0 ! "
                "video/x-raw(memory:NVMM), width=(int)800, height=(int)600, "
                "format=NV12, framerate=(fraction)30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink";
            camera.initLocalEnv(pipeline, "calibration.yml");
            trajectoryDefinition.initLocalEnv();
            laneDetectionFile = "/home/team02/Models/engine/lane_Yolo_Carla_epoch_50.engine";
            objDetectionFile = "/home/team02/Models/engine/obj_MOB_1_epoch_133.engine";
        } else {
            std::cout << "Running in CARLA mode with simulated camera" << std::endl;
            camera.initCarlaEnv();
            trajectoryDefinition.initCarlaEnv();
            laneDetectionFile = "/home/jorge/Downloads/lane_Yolo_Carla_epoch_50.engine";
            objDetectionFile = "/home/jorge/Downloads/obj_MOB_1_epoch_133.engine";
        }

        LaneDetector laneDetector(laneDetectionFile, height, width);
        ObjectDetector objDetector(objDetectionFile, height, width);
    
        camera.startCapture();

        camThread = std::thread(cameraThreadFunction, &camera, &processor);
        laneThread = std::thread(laneDetectionThreadFunction, &laneDetector, &processor);
        objThread = std::thread(objectDetectionThreadFunction, &objDetector, &processor);
        trajThread = std::thread(trajectoryThreadFunction, &trajectoryDefinition, &processor);

        std::cout << "Running. Press Ctrl+C to exit." << std::endl;
        while(running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        std::cout << "Shutting down..." << std::endl;
        processor.shutdown(); 

        std::cout << "Waiting for threads to finish..." << std::endl;
        if (camThread.joinable()) {
            std::cout << "Joining camera thread..." << std::endl;
            camThread.join();
        }
        if (laneThread.joinable()) {
            std::cout << "Joining lane detection thread..." << std::endl;
            laneThread.join();
        }
        if (objThread.joinable()) {
            std::cout << "Joining object detection thread..." << std::endl;
            objThread.join();
        }
        if (trajThread.joinable()) {
            std::cout << "Joining trajectory thread..." << std::endl;
            trajThread.join();
        }

        camera.stopCapture();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

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

        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
        cv::imencode(".jpg", new_frame, buffer, params);
        
        trajectoryDef->frame_publisher_->put(buffer);

        processor->trajectoryDone();
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);

    try
    {
        std::string configFile;
        std::string mode;
        
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
            config.insert_json5("listen/endpoints", "[\"udp/100.117.122.95:7450\"]");
            config.insert_json5("connect/endpoints", "[\"udp/100.117.122.95:7447\"]");
            // config.insert_json5("listen/endpoints", "[\"udp/100.119.72.83:7450\"]");
            // config.insert_json5("connect/endpoints", "[\"udp/100.119.72.83:7447\"]");
            session = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }

        SynchronizedProcessor processor;

        Camera camera(session);
        LaneDetector laneDetector("/home/luis_t2/SEAME/Team02-Course/MachineLearning/LaneDetection/Models/engine/lane_Yolo2_epoch_45.engine");
        ObjectDetector objDetector("/home/luis_t2/SEAME/Team02-Course/MachineLearning/ObjectDetection/Models/engine/obj_MOB_1_epoch_133.engine");
        // LaneDetector laneDetector("/home/team02/Models/engine/lane_Yolo2_epoch_45.engine");
        // ObjectDetector objDetector("/home/team02/Models/engine/obj_MOB_1_epoch_133.engine");
        TrajectoryDefinition trajectoryDefinition(session);

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
        } else {
            std::cout << "Running in CARLA mode with simulated camera" << std::endl;
            camera.initCarlaEnv();
            trajectoryDefinition.initCarlaEnv();
        }
    
        camera.startCapture();

        std::thread camThread(cameraThreadFunction, &camera, &processor);
        std::thread laneThread(laneDetectionThreadFunction, &laneDetector,
                               &processor);
        std::thread objThread(objectDetectionThreadFunction, &objDetector,
                              &processor);
        std::thread trajThread(trajectoryThreadFunction, &trajectoryDefinition,
                               &processor);

        if (camThread.joinable())
        {
            camThread.join();
        }
        if (laneThread.joinable())
        {
            laneThread.join();
        }
        if (objThread.joinable())
        {
            objThread.join();
        }
        if (trajThread.joinable())
        {
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

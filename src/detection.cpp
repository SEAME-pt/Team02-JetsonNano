#include "LaneDetector.hpp"
#include "ObjectDetector.hpp"
#include "TrajectoryDefinition.hpp"
#include "TrafficSignClassifier.hpp"
#include "Camera.hpp"
#include "SynchronizedProcessor.hpp"
#include "utils.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#include <csignal>
#include <cstdlib>

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

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void laneDetectionThreadFunction(LaneDetector* detector,
                                 SynchronizedProcessor* processor)
{
    while (running)
    {
        cv::Mat frame = processor->getLaneFrame();

        if (!frame.empty())
        {
            cv::Mat result;
            detector->detect(frame, result);

            processor->laneDone(result);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void objectDetectionThreadFunction(ObjectDetector* detector,
                                   SynchronizedProcessor* processor)
{
    while (running)
    {
        cv::Mat frame = processor->getObjectFrame();

        if (!frame.empty())
        {
            cv::Mat result;
            detector->detect(frame, result);

            processor->objectDone(result);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void trajectoryThreadFunction(TrajectoryDefinition* trajectoryDef,
                              SynchronizedProcessor* processor)
{
    cv::Mat original_frame, lane_mask, object_mask;

    while (running)
    {
        processor->getProcessingData(original_frame, lane_mask, object_mask);
        if (!original_frame.empty() && !lane_mask.empty() &&
            !object_mask.empty())
        {
            cv::Mat new_frame =
                trajectoryDef->process(original_frame, lane_mask, object_mask);

            std::vector<uchar> buffer_ipm_frame;
            std::vector<int> params_ipm = {cv::IMWRITE_JPEG_QUALITY, 20};
            cv::imencode(".jpg", new_frame, buffer_ipm_frame, params_ipm);
            trajectoryDef->publishIPMFrame(
                std::string(buffer_ipm_frame.begin(), buffer_ipm_frame.end()));

            std::vector<uchar> buffer_original_frame;
            std::vector<int> params_org_frame = {cv::IMWRITE_JPEG_QUALITY, 20};
            cv::imencode(".jpg", original_frame, buffer_original_frame,
                         params_org_frame);
            trajectoryDef->publishOrigFrame(std::string(
                buffer_original_frame.begin(), buffer_original_frame.end()));

            std::vector<uchar> buffer_lane_mask;
            std::vector<int> params_lane = {cv::IMWRITE_JPEG_QUALITY, 20};
            cv::imencode(".jpg", lane_mask, buffer_lane_mask, params_lane);
            trajectoryDef->publishBinMask(
                std::string(buffer_lane_mask.begin(), buffer_lane_mask.end()));

            std::vector<uchar> buffer_obj_mask;
            std::vector<int> params_obj = {cv::IMWRITE_JPEG_QUALITY, 20};
            cv::imencode(".jpg", object_mask, buffer_obj_mask, params_obj);
            trajectoryDef->publishClassMask(
                std::string(buffer_obj_mask.begin(), buffer_obj_mask.end()));

            processor->trajectoryDone();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void trafficSignThreadFunction(TrafficSignClassifier* trafficSignClassifier,
                               SynchronizedProcessor* processor)
{
    cv::Mat frame, object_mask;

    while (running)
    {
        processor->getFrameAndObjectMask(frame, object_mask);

        if (!frame.empty() && !object_mask.empty())
        {
            cv::Mat result;

            trafficSignClassifier->classify(frame, object_mask, result);

            if (!result.empty())
            {
                std::vector<uchar> buffer_trafficSign_frame;
                std::vector<int> params_trafficSign = {cv::IMWRITE_JPEG_QUALITY,
                                                       50};
                cv::imencode(".jpg", result, buffer_trafficSign_frame,
                             params_trafficSign);
                trafficSignClassifier->publishTrafficSignFrame(
                    std::string(buffer_trafficSign_frame.begin(),
                                buffer_trafficSign_frame.end()));
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    std::thread camThread, laneThread, objThread, trajThread, trafficSignThread;

    const int heightCameraFrame     = 480;
    const int widthCameraFrame      = 640;
    const int heightModelInf        = 128;
    const int widthModelInf         = 256;
    const int heightTrafficModelInf = 128;
    const int widthTrafficModelInf  = 128;

    try
    {
        std::string configFile;
        std::string mode;
        std::string laneDetectionFile;
        std::string objDetectionFile;
        std::string trafficClassifierFile;

        if (!parseParameters(argc, argv, configFile, mode))
        {
            return -1;
        }

        mode = "local";
        std::shared_ptr<zenoh::Session> session;
        if (!configFile.empty())
        {
            std::cout << "Using configuration from file: " << configFile
                      << std::endl;
            auto config = Config::from_file(configFile);
            session     = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }
        else
        {
            std::cout << "Using default configuration" << std::endl;
            auto config = Config::create_default();
            // config.insert_json5("listen/endpoints",
            // "[\"udp/100.73.255.97:7450\"]");
            // config.insert_json5("connect/endpoints",
            // "[\"udp/100.73.255.97:7447\"]");
            // config.insert_json5("listen/endpoints",
            // "[\"udp/100.117.122.95:7450\"]");
            // config.insert_json5("connect/endpoints",
            // "[\"udp/100.117.122.95:7447\"]");
            config.insert_json5("listen/endpoints",
                                "[\"udp/100.119.72.83:7450\"]");
            config.insert_json5("connect/endpoints",
                                "[\"udp/100.119.72.83:7447\"]");
            session = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }

        SynchronizedProcessor processor;
        Camera camera(session);
        TrajectoryDefinition trajectoryDefinition(session, heightCameraFrame,
                                                  widthCameraFrame);

        if (mode == "local")
        {
            std::cout << "Running in LOCAL mode with physical camera"
                      << std::endl;
            const std::string pipeline =
                "nvarguscamerasrc sensor-id=0 ! "
                "video/x-raw(memory:NVMM), width=(int)640, height=(int)480, "
                "format=NV12, framerate=(fraction)30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink";
            camera.initLocalEnv(pipeline,
                                "/home/team02/Team02-Course/JetsonNano/tools/"
                                "cam_calibration/calibration.yml");
            trajectoryDefinition.initLocalEnv();
            laneDetectionFile =
                "/home/team02/Models/engine/"
                "lane_Mob_local_pretrained_tusimple5_epoch_50_2.engine";
            objDetectionFile =
                "/home/team02/Models/engine/"
                "obj_Mob_local_pretrained_BDD100k3_epoch_100.engine";
            trafficClassifierFile =
                "/home/team02/Models/engine/traffic_sign_model5.engine";
        }
        else if (mode == "carla")
        {
            std::cout << "Running in CARLA mode with simulated camera"
                      << std::endl;
            camera.initCarlaEnv();
            trajectoryDefinition.initCarlaEnv();
            laneDetectionFile =
                "/home/jorge/Downloads/lane_Yolo_Carla3_epoch_16.engine";
            objDetectionFile =
                "/home/jorge/Downloads/obj_YOLO_Carla1_epoch_200.engine";
            trafficClassifierFile =
                "/home/team02/Models/engine/"
                "obj_Mob_local_pretrained_BDD100k1_epoch_100.engine";
        }
        else
        {
            std::cout << "Running in TEST mode with test video" << std::endl;
            const std::string video = "/home/team02/record_cam/build/video.mp4";
            camera.initLVideoTestEnv(video);
            trajectoryDefinition.initLocalEnv();
            laneDetectionFile =
                "/home/team02/Models/engine/"
                "lane_Mob_local_pretrained_tusimple4_epoch_30.engine";
            objDetectionFile =
                "/home/team02/Models/engine/"
                "obj_Mob_local_pretrained_BDD100k1_epoch_100.engine";
            trafficClassifierFile =
                "/home/team02/Models/engine/traffic_sign_model.engine";
        }

        LaneDetector laneDetector(laneDetectionFile, heightModelInf,
                                  widthModelInf);
        ObjectDetector objDetector(objDetectionFile, heightModelInf,
                                   widthModelInf);
        TrafficSignClassifier trafficSignClassifier(
            trafficClassifierFile, session, heightTrafficModelInf,
            widthTrafficModelInf);

        camera.startCapture();

        camThread = std::thread(cameraThreadFunction, &camera, &processor);
        laneThread =
            std::thread(laneDetectionThreadFunction, &laneDetector, &processor);
        objThread  = std::thread(objectDetectionThreadFunction, &objDetector,
                                 &processor);
        trajThread = std::thread(trajectoryThreadFunction,
                                 &trajectoryDefinition, &processor);
        trafficSignThread = std::thread(trafficSignThreadFunction,
                                        &trafficSignClassifier, &processor);

        std::cout << "Running. Press Ctrl+C to exit." << std::endl;
        while (running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "Shutting down..." << std::endl;
        processor.shutdown();

        std::cout << "Waiting for threads to finish..." << std::endl;
        if (camThread.joinable())
        {
            std::cout << "Joining camera thread..." << std::endl;
            camThread.join();
        }
        if (laneThread.joinable())
        {
            std::cout << "Joining lane detection thread..." << std::endl;
            laneThread.join();
        }
        if (objThread.joinable())
        {
            std::cout << "Joining object detection thread..." << std::endl;
            objThread.join();
        }
        if (trajThread.joinable())
        {
            std::cout << "Joining trajectory thread..." << std::endl;
            trajThread.join();
        }
        if (trafficSignThread.joinable())
        {
            std::cout << "Joining traffic classifier thread..." << std::endl;
            trafficSignThread.join();
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

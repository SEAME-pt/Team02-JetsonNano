#include "LaneDetector.hpp"
#include "ObjectDetector.hpp"
#include "TrajectoryDefinition.hpp"
#include "Camera.hpp"
#include <thread>
#include <atomic>
#include <condition_variable> // Add this
#include <queue>              // Add this
#include <memory>             // Add this
#include <iostream>
#include <csignal>

using namespace cv;
using namespace std;
using namespace zenoh;

std::atomic<bool> running(true);

class SynchronizedProcessor
{
  private:
    std::mutex sync_mutex;
    std::condition_variable inference_cv;
    std::condition_variable trajectory_cv;
    std::condition_variable camera_cv;

    cv::Mat current_frame;
    cv::Mat lane_binary_mask;
    cv::Mat object_class_mask;

    bool lane_ready          = true;
    bool object_ready        = true;
    bool trajectory_done     = true;
    bool new_frame_available = false;
    int frame_id             = 0;

  public:
    void setNewFrame(const cv::Mat& frame)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        camera_cv.wait(
            lock,
            [this]() { return lane_ready && object_ready && trajectory_done; });

        frame.copyTo(current_frame);

        lane_ready          = false;
        object_ready        = false;
        new_frame_available = true;

        inference_cv.notify_all();
    }

    // Lane detection thread calls this
    cv::Mat getLaneFrame()
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        inference_cv.wait(lock, [this]()
                          { return new_frame_available && !lane_ready; });

        return current_frame.clone();
    }

    cv::Mat getObjectFrame()
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        inference_cv.wait(lock, [this]()
                          { return new_frame_available && !object_ready; });

        return current_frame.clone();
    }

    void laneDone(const cv::Mat& result)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        result.copyTo(lane_binary_mask);
        lane_ready = true;

        checkBothDone();
    }

    // Object thread calls this when done
    void objectDone(const cv::Mat& result)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        result.copyTo(object_class_mask);
        object_ready = true;

        checkBothDone();
    }

    void getProcessingData(cv::Mat& original, cv::Mat& lane_mask,
                           cv::Mat& object_mask)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);

        trajectory_cv.wait(
            lock, [this]()
            { return lane_ready && object_ready && !trajectory_done; });

        current_frame.copyTo(original);
        lane_binary_mask.copyTo(lane_mask);
        object_class_mask.copyTo(object_mask);
    }

    void trajectoryDone()
    {
        std::unique_lock<std::mutex> lock(sync_mutex);
        trajectory_done = true;

        camera_cv.notify_one();
    }

  private:
    void checkBothDone()
    {
        if (lane_ready && object_ready)
        {
            trajectory_done     = false;
            new_frame_available = false;
            trajectory_cv.notify_one();
        }
    }
};

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


// Base64 encoding function
static const std::string base64_chars = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (in_len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for(i = 0; (i < 4); i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for(j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];

        while((i++ < 3))
            ret += '=';
    }

    return ret;
}

void trajectoryThreadFunction(TrajectoryDefinition* trajectoryDef,
                              SynchronizedProcessor* processor)
{   
    cv::Mat original_frame, lane_mask, object_mask;

    while (running)
    {
        processor->getProcessingData(original_frame, lane_mask, object_mask);

        cv::Mat new_frame = trajectoryDef->process(original_frame, lane_mask, object_mask);

            // Encode new_frame as JPEG
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85}; // Adjust quality as needed
        cv::imencode(".jpg", new_frame, buffer, params);
        
        // Base64 encode the image data
        std::string base64_data = base64_encode(buffer.data(), buffer.size());
        
        // Publish the encoded frame data
        trajectoryDef->frame_publisher_->put(base64_data);

        processor->trajectoryDone();
    }
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
            config.insert_json5("listen/endpoints", "[\"udp/100.117.122.95:7450\"]");
            config.insert_json5("connect/endpoints", "[\"udp/100.117.122.95:7447\"]");
            session     = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }

        // const std::string pipeline =
        //     "nvarguscamerasrc sensor-id=0 ! "
        //     "video/x-raw(memory:NVMM), width=(int)640, height=(int)480, "
        //     "format=NV12, framerate=(fraction)30/1 ! "
        //     "nvvidconv ! video/x-raw, format=BGRx ! "
        //     "videoconvert ! video/x-raw, format=BGR ! "
        //     "appsink";

        SynchronizedProcessor processor;
    
        Camera camera(session);
        LaneDetector laneDetector("/home/luis_t2/SEAME/Team02-Course/MachineLearning/LaneDetection/Models/engine/lane_Yolo2_epoch_45.engine");
        ObjectDetector objDetector("/home/luis_t2/SEAME/Team02-Course/MachineLearning/ObjectDetection/Models/engine/obj_MOB_1_epoch_133.engine");
        TrajectoryDefinition trajectoryDefinition(session);

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

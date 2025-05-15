#include "ObjectDetector.hpp"

using namespace cv;
using namespace std;
using namespace zenoh;

int main(int argc, char** argv)
{
    const std::string pipeline =
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=(int)640, height=(int)480, "
        "format=NV12, framerate=(fraction)30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink";
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

        std::string file = "/home/team02/obj_MOB_1_epoch_133.engine";
        ObjectDetector detector(file, pipeline, session );
        detector.setCalibrationParameters();
        detector.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

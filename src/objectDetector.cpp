#include "ObjectDetector.hpp"

using namespace cv;
using namespace std;
using namespace zenoh;

int main(int argc, char** argv)
{
    const std::string pipeline =
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=(int)800, height=(int)600, "
        "format=NV12, framerate=(fraction)30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink";
    try
    {
        std::string file = "/home/team02/obj_UNet_3_epoch_185.engine";
        ObjectDetector detector(file, pipeline, session);
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

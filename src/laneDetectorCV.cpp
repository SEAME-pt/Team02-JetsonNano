#include "LaneDetectorCV.hpp"
#include <zenoh.hxx>
#include <iostream>

using namespace zenoh;

int main(void)
{

    //calibration parameters file path here this should be saved on the class to be used in RUN method (function)
    FileStorage fs("calibration.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open calibration.yml" << endl;
        return -1;
    }
    Mat cameraMatrix, distCoeffs;
    fs["CameraMatrix"] >> cameraMatrix;
    fs["DistCoeffs"] >> distCoeffs;
    fs.release();

    const std::string pipeline =
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, "
        "framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink";

    try
    {
        auto config = Config::create_default();
        auto session = std::make_shared<Session>(Session::open(std::move(config)));

        LaneDetectorCV detector(pipeline, session);
        detector.setCalibrationParameters(cameraMatrix, distCoeffs);
        detector.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
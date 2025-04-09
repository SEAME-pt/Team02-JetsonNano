#include "LaneDetector.hpp"

using namespace cv;
using namespace std;
using namespace zenoh;

int main(int argc, char** argv)
{

    // Make the pipeline object with the streamer
    OpencvGStreamerPipeline opencvGSpipeline("nvarguscamerasrc");
    opencvGSpipeline.setStreamerSettings(streamerSourceParams);

    // Add a sink and a source to process the sink
    opencvGSpipeline.addSink("nvvidconv");
    opencvGSpipeline.addSource("video/x-raw, format=(string)BGRx");

    // Add a sink and source together
    std::string secondSink = "videoconvert";
    std::string secondSource = "video/x-raw, format=(string)BGR";
    opencvGSpipeline.addElement(secondSink, secondSource);

    // Get the pipeline string to pass to opencv
    std::string GSpipeline = opencvGSpipeline.getPipelineString();
    std::cout << "G-Streamer pipeline is: " << GSpipeline << std::endl;

    const std::string pipeline =
        "nvarguscamerasrc !
         sensor-id=0 !
         video/x-raw(memory:NVMM), width=(int)216, height=(int)128, format=NV12, 
        framerate=(fraction)30/1 !
        nvvidconv ! video/x-raw, format=BGRx !
        videoconvert ! video/x-raw, format=BGR !
        appsink";

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
        std::string file = "/home/team02/cam/model_segmentation10.engine";
        LaneDetector detector(file, pipeline, session);
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

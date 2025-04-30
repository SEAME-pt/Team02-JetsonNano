#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <ctime>

int main() {
    // GStreamer pipeline for the CSI camera on CAM0 (sensor-id=0)
    std::string pipeline =
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink";

    // Open the camera using the GStreamer pipeline
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the CSI camera on CAM0." << std::endl;
        return -1;
    }

    std::cout << "Press space to capture an image, 'q' to quit." << std::endl;

    cv::Mat frame;
    while (true) {
        // Capture frame
        if (!cap.read(frame)) {
            std::cerr << "Error: Unable to receive frame. Exiting..." << std::endl;
            break;
        }

        // Display the camera feed in a window
        cv::imshow("Camera Feed", frame);

        // Wait 1ms for key input
        char key = (char)cv::waitKey(1);
        if (key == ' ') { // Capture image on space key press
            time_t now = time(0);
            char timestamp[20];
            strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", localtime(&now));
            // Save each captured image in the imgs directory
            std::string filename = std::string("imgs/new") + timestamp + ".jpg";
            if (cv::imwrite(filename, frame)) {
                std::cout << "Image saved as " << filename << std::endl;
            } else {
                std::cerr << "Error: Could not save image." << std::endl;
            }
        } else if (key == 'q') { // Quit on 'q'
            break;
        }
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}


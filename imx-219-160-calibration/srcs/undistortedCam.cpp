#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main() {

    // If you run the executable from another location, update this path.
    FileStorage fs("calibration.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open calibration.yml" << endl;
        return -1;
    }
    Mat cameraMatrix, distCoeffs;
    fs["CameraMatrix"] >> cameraMatrix;
    fs["DistCoeffs"] >> distCoeffs;
    fs.release();

    // Define the GStreamer pipeline for the CSI camera on the Jetson Nano
    string pipeline =
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink";

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open the CSI camera." << endl;
        return -1;
    }
    namedWindow("Undistorted Feed", WINDOW_AUTOSIZE);

    Mat frame, undistorted;
    while (true) {
        // Capture a frame from the camera
        if (!cap.read(frame)) {
            cerr << "Error: Unable to capture frame." << endl;
            break;
        }
        // Apply the undistortion using the calibration parameters
        undistort(frame, undistorted, cameraMatrix, distCoeffs);
        imshow("Undistorted Feed", undistorted);

        // Break the loop if 'q' or ESC is pressed
        char key = (char)waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }
    cap.release();
    destroyAllWindows();
    return 0;
}

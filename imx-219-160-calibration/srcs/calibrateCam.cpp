#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

int main() {

    // corners per column (inside the inner chessboard borders)
    int boardWidth = 9; 
    int boardHeight = 6; 
    Size boardSize(boardWidth, boardHeight);

    // Square size in mm of the chessboard calibration pattern
    float squareSize = 25.0f;

    // Prepare object points for one calibration view
    vector<Point3f> obj;
    for (int i = 0; i < boardHeight; i++) {
        for (int j = 0; j < boardWidth; j++) {
            obj.push_back(Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;

    //might want to change the images that are used for calibration here
    // Process images j10.jpg to j22.jpg located in the imgs directory
    vector<string> imageFiles;
    for (int i = 10; i <= 22; i++) {
        stringstream ss;
        ss << "../imgs/j" << i << ".jpg";
        imageFiles.push_back(ss.str());
    }

    for (size_t i = 0; i < imageFiles.size(); i++) {
        Mat image = imread(imageFiles[i]);
        if (image.empty()) {
            cout << "Could not open or find image: " << imageFiles[i] << endl;
            continue;
        }

        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners;
        // Find the chessboard corners
        bool found = findChessboardCorners(gray, boardSize, corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            // Refine corner locations for more accurate calibration
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001));
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
            // Draw the corners on the image for visual verification
            drawChessboardCorners(image, boardSize, corners, found);
            imshow("Detected Corners", image);
            // Display the image for 500ms just to check change this  to 1 or 0 if you just want to  skip
            waitKey(500);
        } else {
            cout << "Chessboard corners not found in " << imageFiles[i] << endl;
        }
    }
    destroyAllWindows();

    if (imagePoints.empty() || objectPoints.empty()) {
        cerr << "Calibration failed: No valid images were processed." << endl;
        return -1;
    }
    // Use the size of the first image for calibration
    Mat temp = imread(imageFiles[0]);
    Size imageSize = temp.size();

    // Calculate    matrix and distortion coefficients
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize,
                                 cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "Calibration done with RMS error = " << rms << endl;
    cout << "Camera Matrix:\n" << cameraMatrix << endl;
    cout << "Distortion Coefficients:\n" << distCoeffs << endl;

    // Save the calibration
    FileStorage fs("calibration.yml", FileStorage::WRITE);
    fs << "CameraMatrix" << cameraMatrix;
    fs << "DistCoeffs" << distCoeffs;
    fs.release();

    return 0;
}

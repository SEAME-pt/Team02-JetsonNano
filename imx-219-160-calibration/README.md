# IMX 219-160 Camera Calibration

This repository contains code for calibrating an IMX219 camera (commonly used with the Jetson Nano) and tools for capturing and viewing undistorted images.

## Contents

* `build/`: Directory for build artifacts.
* `imgs/`: Contains chessboard images used for calibration.
* `srcs/`: Source files for the calibration and image processing tools.
* `CMakeLists.txt`: CMake build configuration file.
* `instrunctions.txt`: (Likely contains similar information to this README).

## Tools

The repository builds the following executables:

* **`pictureTaker`**: A simple application to capture images by pressing the spacebar. This is built for the Jetson Nano and might require adjustments for other platforms. The captured images are saved in the current directory.
* **`calibrateCam`**: This tool uses the chessboard images in the `imgs/` directory to generate a camera calibration file (`calibration.yml`).
* **`undistortedCam`**: This application loads the `calibration.yml` file and displays a live, undistorted view from the camera.

## Setup and Usage

Follow these steps to build and use the tools:

1.  **Create a build directory:**
    ```bash
    mkdir build
    cd build
    ```

2.  **Configure the project using CMake:**
    ```bash
    cmake ..
    ```

3.  **Build the executables:**
    ```bash
    make
    ```

4.  **Run the tools (from the `build` directory):**

    * **Taking Pictures:**
        ```bash
        ./pictureTaker
        ```
        Press the spacebar to capture images.

    * **Calibrating the Camera:**
        ```bash
        ./calibrateCam
        ```
        This will process the chessboard images in the `../imgs/` directory and generate `calibration.yml` in the `build/` directory.

    * **Viewing Undistorted Camera Feed:**
        ```bash
        ./undistortedCam
        ```
        This will open a window showing the live camera feed with distortion correction applied using the `calibration.yml` file.

## Calibration Details

* This calibration process uses chessboard images. The provided images (`imgs/j10.jpg` to `imgs/j22.jpg`) are likely from the OpenCV website.
* **Important:** For accurate calibration, you need to set the correct parameters within the `srcs/calibrateCam.cpp` file:
    * `boardWidth`: The number of inner corners along the width of the chessboard. (Currently set to 9)
    * `boardHeight`: The number of inner corners along the height of the chessboard. (Currently set to 6)
    * `squareSize`: The size of each square on the chessboard in millimeters. (Currently set to 25.0f)
* **Calibration Images:** It is highly recommended to capture your own set of chessboard images using your specific camera. Ensure the chessboard is clearly visible and captured from various angles and distances. Replace the images in the `imgs/` directory with your own for a more accurate calibration.

## Optimization

For improved performance, especially in real-time applications, you can initialize a calibration map using the calculated camera matrix and distortion coefficients. This map can then be used with the `cv::remap()` function to efficiently undistort incoming frames, significantly increasing processing speed. This optimization is not explicitly implemented in the provided `undistortedCam.cpp` but is a recommended next step for enhancing the application.

```cpp
// Example of initializing a remap for undistortion (not in the provided code)
cv::Mat map1, map2;
cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);

// Then, in your video processing loop:
// cv::remap(frame, undistortedFrame, map1, map2, cv::INTER_LINEAR);

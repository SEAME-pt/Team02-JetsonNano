# IMX 219-160 Camera Calibration

This directory contains code for calibrating an IMX219 camera (commonly used with the Jetson Nano) and tools for capturing and viewing undistorted images.

## Contents

* `build/`: Directory for build artifacts.
* `imgs/`: Contains chessboard images used for calibration.
* `srcs/`: Source files for the calibration and image processing tools.
* `CMakeLists.txt`: CMake build configuration file.
* `instrunctions.txt`: (Likely contains similar information to this README).

## Tools

The directory builds the following executables:

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
```

---

# Project Requirements

## Compiler
- **g++** version 9.0 or higher  
  (Supports C++17 features such as `<filesystem>` which might be used)

- **cmake** version 3.10 or higher  
  (For building and managing your project)

## Libraries
- **OpenCV** version 4.0 or higher  
  - Must be built with **GStreamer** support enabled (Jetson Nano’s preinstalled OpenCV has this by default)  
  - Provides core computer vision functions and camera calibration tools

- **GStreamer** version 1.14 or higher  
  - Used for accessing the CSI camera stream via a GStreamer pipeline on Jetson Nano  
  - Packages to install:  
    - `libgstreamer1.0-dev`  
    - `libgstreamer-plugins-base1.0-dev`

## Build Tools
- **make** (usually installed with build-essential packages)  
- Optional: **ninja** (faster alternative build system)

## C++ Standard
- Use **C++17** or later  
  - Required for features like `<filesystem>` (used to create directories automatically)

## Platform Notes
- **Jetson Nano** comes with GStreamer and OpenCV preinstalled and configured correctly by default.  
- On other Linux systems, ensure OpenCV is compiled with the `-D WITH_GSTREAMER=ON` flag if you want GStreamer support.  
- On Windows or macOS, consider using package managers like **vcpkg** or **conan** to install OpenCV with GStreamer support.

---

## Installing Dependencies on Jetson Nano (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y \
    g++ cmake make \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev
```

cmake_minimum_required(VERSION 3.10)
project(Cam CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Build calibrateCam this will create a calibration file once run.
add_executable(calibrateCam srcs/calibrateCam.cpp)
target_link_libraries(calibrateCam ${OpenCV_LIBS})

# Build undistortedCam viewer (testing the calibration)
add_executable(undistortedCam srcs/undistortedCam.cpp)
target_link_libraries(undistortedCam ${OpenCV_LIBS})

# Build pictureTaker
add_executable(pictureTaker srcs/pictureTaker.cpp)
target_link_libraries(pictureTaker ${OpenCV_LIBS})

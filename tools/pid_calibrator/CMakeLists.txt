cmake_minimum_required(VERSION 3.16)
project(Calibrator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

add_executable(pid_calibrator
    ./src/main.cpp
    ./src/Communication/Publisher.cpp
    ./src/pidCalibrator.cpp
)

target_include_directories(pid_calibrator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Communication
)

target_link_libraries(pid_calibrator PUBLIC
    zenohcxx::zenohc
)
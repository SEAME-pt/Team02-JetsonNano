cmake_minimum_required(VERSION 3.16)

project(PCA9685)

add_library(PCA9685
    src/PCA9685.cpp
)

target_include_directories(PCA9685 PUBLIC include)
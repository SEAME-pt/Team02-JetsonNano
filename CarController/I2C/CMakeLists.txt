cmake_minimum_required(VERSION 3.16)

project(CarController VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    ./src/I2C.cpp
    ./src/PCA9685.cpp
    ./src/main.cpp
)

set(HEADERS 
    ./include/I2C.hpp
    ./include/PCA9685.hpp
)

add_executable(i2c
    ${HEADERS} ${SOURCES}
)
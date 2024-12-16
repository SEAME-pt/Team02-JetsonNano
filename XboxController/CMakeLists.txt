cmake_minimum_required(VERSION 3.16)

project(CarController VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXE_LINKER_FLAGS "-static")

include_directories(include)
include_directories(../SharedMemory)

set(SOURCES
    ./src/main.cpp
    ./src/XboxController.cpp
)

add_executable(controller
    ${SOURCES}
)

cmake_minimum_required(VERSION 3.16)

project(CarController VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXE_LINKER_FLAGS "-static")

include_directories(include)
include_directories(../SharedMemory)

add_subdirectory(libs/Communication/I2C)
add_subdirectory(libs/Peripherals/PCA9685)

set(SOURCES
    ./src/main.cpp
    ./src/RaceCar.cpp
    ./src/Motor.cpp
    ./src/Servo.cpp
)

add_executable(RaceCar
    ${SOURCES}
)

target_compile_options(RaceCar PRIVATE -Wall -Werror -Wextra)

target_link_libraries(RaceCar I2C PCA9685)
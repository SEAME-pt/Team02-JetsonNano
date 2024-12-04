cmake_minimum_required(VERSION 3.16)

project(CarController VERSION 0.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

set(CMAKE_EXE_LINKER_FLAGS "-static")

add_subdirectory(external/I2C)
add_subdirectory(external/PCA9685)

set(SOURCES
    ./src/main.cpp
    ./src/RaceCar.cpp
)

add_executable(i2c
    ${SOURCES}
)

target_link_libraries(i2c I2C PCA9685)
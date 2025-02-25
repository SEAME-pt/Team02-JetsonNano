cmake_minimum_required(VERSION 3.16)

project(CarController VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXE_LINKER_FLAGS "-static")
set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

include(FetchContent)
FetchContent_Declare(
    Libs
    GIT_REPOSITORY https://github.com/SEAME-pt/Team02-Libs
    GIT_TAG main
)
FetchContent_MakeAvailable(Libs)    

include_directories(include)
include_directories(${libs_SOURCE_DIR}/Communication/I2C/include)
include_directories(${libs_SOURCE_DIR}/Communication/CAN/include)
include_directories(${libs_SOURCE_DIR}/Peripherals/INA219/include)

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

target_link_libraries(RaceCar
    I2C
    PCA9685
    zenohcxx::zenohc)

###TESTS###

target_link_libraries(
    I2C
    PCA9685
    zenohcxx::zenohc
)

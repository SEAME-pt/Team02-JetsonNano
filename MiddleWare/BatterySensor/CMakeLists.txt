cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

project(BatterySensor)

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

include_directories(${libs_SOURCE_DIR}/Communication/I2C/include)
include_directories(${libs_SOURCE_DIR}/Communication/CAN/include)
include_directories(${libs_SOURCE_DIR}/Peripherals/INA219/include)
include_directories(include)

add_library(BatterySensor
    src/BatterySensor.cpp
)

target_link_libraries(BatterySensor PUBLIC zenohcxx::zenohc INA219 CAN I2C)
cmake_minimum_required(VERSION 3.16)

project(CarController VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXE_LINKER_FLAGS "-static")
set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

add_subdirectory(libs/Communication/I2C)
add_subdirectory(libs/Peripherals/PCA9685)


include_directories(include)

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

target_link_libraries(RaceCar I2C PCA9685 zenohcxx::zenohc)


###TESTS###

add_subdirectory(../external/googletest)

include_directories(../external/googletest/googletest/include)
include_directories(tests/unit/mocks)

add_executable(tests 
    ./tests/I2CTest.cpp
    ./tests/PCA9685Test.cpp
    ./tests/RaceCarTest.cpp
)

target_link_libraries(tests gtest gtest_main I2C PCA9685)

enable_testing()
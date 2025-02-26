cmake_minimum_required(VERSION 3.16)

project(CarController VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_EXE_LINKER_FLAGS "-static")
# set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
# set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

# find_package(zenohc REQUIRED)
# find_package(zenohcxx REQUIRED)

# include(FetchContent)
# FetchContent_Declare(
#     Libs
#     GIT_REPOSITORY https://github.com/SEAME-pt/Team02-Libs
#     GIT_TAG main
# )
# FetchContent_MakeAvailable(Libs)    

include_directories(include)
include_directories(include/ADAS)
include_directories(include/Body)
include_directories(include/Chassis)
include_directories(include/Powertrain)
include_directories(include/Vehicle)
include_directories(include/Vehicle/MotionManagement)
# include_directories(${libs_SOURCE_DIR}/Communication/I2C/include)
# include_directories(${libs_SOURCE_DIR}/Communication/CAN/include)
# include_directories(${libs_SOURCE_DIR}/Peripherals/INA219/include)

set(SOURCES
    ./src/ADAS/ADAS.cpp
    ./src/ADAS/ObstacleDetection.cpp
    ./src/Body/Body.cpp
    ./src/Body/Lights.cpp
    ./src/Chassis/Accelerator.cpp
    ./src/Chassis/Axle.cpp
    ./src/Chassis/Brake.cpp
    ./src/Chassis/Chassis.cpp
    ./src/Chassis/ChassisSteeringWheel.cpp
    ./src/Powertrain/ElectricMotor.cpp
    ./src/Powertrain/Powertrain.cpp
    ./src/Powertrain/Transmission.cpp
    ./src/Vehicle/Acceleration.cpp
    ./src/Vehicle/AngularVelocity.cpp
    ./src/Vehicle/Connectivity.cpp
    ./src/Vehicle/Exterior.cpp
    ./src/Vehicle/MotionManagement/ElectricAxle.cpp
    ./src/Vehicle/MotionManagement/MotionManagement.cpp
    ./src/Vehicle/MotionManagement/Steering.cpp
    ./src/Vehicle/MotionManagement/SteeringWheel.cpp
    ./src/Vehicle.cpp
    ./src/Main.cpp
)

add_executable(RaceCar
    ${SOURCES}
)

target_compile_options(RaceCar PRIVATE -Wall -Werror -Wextra)

# target_link_libraries(RaceCar
#     I2C
#     PCA9685
#     zenohcxx::zenohc)

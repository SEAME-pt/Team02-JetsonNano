cmake_minimum_required(VERSION 3.16)

project(VehicleSystem VERSION 0.1)

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

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(Catch2)

include_directories(include/Communication)
include_directories(include/Controllers)
include_directories(include/Hardware)
include_directories(include/ADAS)
include_directories(include/Body)
include_directories(include/Chassis)
include_directories(include/Powertrain)
include_directories(include/Vehicle)
include_directories(include/Vehicle/MotionManagement)
include_directories(include/MiddleWare/BatterySensor)
include_directories(include/MiddleWare/Signals)
include_directories(${libs_SOURCE_DIR}/Communication/I2C/include)
include_directories(${libs_SOURCE_DIR}/Communication/CAN/include)
include_directories(${libs_SOURCE_DIR}/Peripherals/INA219/include)

set(SOURCES
    ./src/Communication/VSSSubscriber.cpp
    ./src/Communication/VSSQueryable.cpp
    ./src/Hardware/MotorController.cpp
    ./src/Hardware/ServoController.cpp
    ./src/Hardware/HardwareObserver.cpp
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
    ./src/Powertrain/TractionBattery.cpp
    ./src/Vehicle/Acceleration.cpp
    ./src/Vehicle/AngularVelocity.cpp
    ./src/Vehicle/Connectivity.cpp
    ./src/Vehicle/Vehicle.cpp
    ./src/Vehicle/VehicleSystem.cpp
    ./src/Vehicle/Exterior.cpp
    ./src/Vehicle/MotionManagement/ElectricAxle.cpp
    ./src/Vehicle/MotionManagement/MotionManagement.cpp
    ./src/Vehicle/MotionManagement/Steering.cpp
    ./src/Vehicle/MotionManagement/SteeringWheel.cpp
)

add_library(vehicle_lib STATIC
    ${SOURCES}
)

target_link_libraries(vehicle_lib PUBLIC
    zenohcxx::zenohc
    I2C
    PCA9685
    CAN
)

add_executable(VehicleSystem
    ./src/vehicle.cpp
)

target_compile_options(VehicleSystem PRIVATE -Wall -Werror -Wextra)

target_link_libraries(VehicleSystem
    vehicle_lib
)

add_executable(XboxController
    ./src/Controllers/XboxController.cpp
    ./src/Communication/ControllerPublisher.cpp
    ./src/control.cpp
)

target_compile_options(XboxController PRIVATE -Wall -Werror -Wextra)

target_link_libraries(XboxController
    zenohcxx::zenohc)


add_executable(MiddleWare
    ./src/Communication/SensoringPublisher.cpp
    ./src/MiddleWare/BatterySensor/BatterySensor.cpp
    ./src/MiddleWare/Signals/Signals.cpp
    ./src/middleware.cpp
)

target_compile_options(MiddleWare PRIVATE -Wall -Werror -Wextra)

target_link_libraries(MiddleWare
    INA219
    vehicle_lib
)

# TESTING #
enable_testing()
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)

if(ENABLE_COVERAGE)
    include(cmake/CodeCoverage.cmake)
    append_coverage_compiler_flags()
endif()

# Group test files by component
set(VEHICLE_TEST_FILES
    ./tests/Vehicle/VehicleTests.cpp
    ./tests/Vehicle/VehicleSystemTests.cpp
    ./tests/Vehicle/ExteriorTests.cpp
    ./tests/Vehicle/ConnectivityTests.cpp
    ./tests/Vehicle/AngularVelocityTests.cpp
    ./tests/Vehicle/AccelerationTests.cpp
)

set(MOTION_TEST_FILES
    ./tests/Vehicle/MotionManagement/ElectricAxleTests.cpp
    ./tests/Vehicle/MotionManagement/SteeringWheelTests.cpp
    ./tests/Vehicle/MotionManagement/SteeringTests.cpp
    ./tests/Vehicle/MotionManagement/MotionManagementTests.cpp
)

set(POWERTRAIN_TEST_FILES
    ./tests/Powertrain/PowertrainTests.cpp
    ./tests/Powertrain/ElectricMotorTests.cpp
    ./tests/Powertrain/TransmissionTests.cpp
    ./tests/Powertrain/TractionBatteryTests.cpp
)

set(CHASSIS_TEST_FILES
    ./tests/Chassis/ChassisTests.cpp
    ./tests/Chassis/AcceleratorTests.cpp
    ./tests/Chassis/AxleTests.cpp
    ./tests/Chassis/BrakeTests.cpp
    ./tests/Chassis/ChassisSteeringWheelTests.cpp
)

set(BODY_TEST_FILES
    ./tests/Body/BodyTests.cpp
    ./tests/Body/LightsTests.cpp
)

set(ADAS_TEST_FILES
    ./tests/ADAS/ADASTests.cpp
    ./tests/ADAS/ObstacleDetectionTests.cpp
)

# Create test executable
add_executable(vehicle_tests
    ${VEHICLE_TEST_FILES}
    ${MOTION_TEST_FILES}
    ${POWERTRAIN_TEST_FILES}
    ${CHASSIS_TEST_FILES}
    ${BODY_TEST_FILES}
    ${ADAS_TEST_FILES}
)

target_link_libraries(vehicle_tests PRIVATE
    Catch2::Catch2WithMain
    vehicle_lib
)

# Add coverage targets if enabled
if(ENABLE_COVERAGE)
    setup_target_for_coverage_gcovr_html(
        NAME coverage
        EXECUTABLE ctest -j ${PROCESSOR_COUNT}
        DEPENDENCIES vehicle_tests
    )
endif()

include(Catch)
catch_discover_tests(vehicle_tests)
cmake_minimum_required(VERSION 3.10)

project(MiddleWare LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

add_subdirectory(BatterySensor)
add_subdirectory(Signals)

include_directories(BatterySensor/include)
include_directories(Signals/include)

add_executable(middlewareApp
    ./src/main.cpp
)

target_link_libraries(middlewareApp BatterySensor Signals zenohcxx::zenohc)

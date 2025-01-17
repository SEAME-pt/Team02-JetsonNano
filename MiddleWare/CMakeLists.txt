cmake_minimum_required(VERSION 3.10)

project(Middelware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

add_executable(middleWare
    ./src/main.cpp
)

target_link_libraries(middleWare PRIVATE zenohcxx::zenohc)

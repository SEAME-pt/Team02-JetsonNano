cmake_minimum_required(VERSION 3.10)
project(system_monitor)

# Require C++17 or later
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXE_LINKER_FLAGS "-static")
set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

# Add the current directory to the include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add the system_monitor executable from main.cpp and system_monitor.cpp
add_executable(system_monitor main.cpp system_monitor.cpp)

target_link_libraries(system_monitor PUBLIC
    zenohcxx::zenohc
)

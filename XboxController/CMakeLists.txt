cmake_minimum_required(VERSION 3.16)

project(CarController VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXE_LINKER_FLAGS "-static")
set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

include_directories(include)

set(SOURCES
    ./src/main.cpp
    ./src/XboxController.cpp
)

add_executable(Controller
    ${SOURCES}
)

# target_compile_options(Controller PRIVATE -Wall -Werror -Wextra)

target_link_libraries(Controller zenohcxx::zenohc)
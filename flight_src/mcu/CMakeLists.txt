cmake_minimum_required(VERSION 3.10)

project(flight_emu_src)

file(GLOB lib_src lib/teensyShared/*.cpp lib/FifoBuffer/*.cpp lib/ServoControl/*.cpp)

add_subdirectory(lib/eigen)

add_library(flight_emu ${lib_src} lib/emulation/EMU.cpp)

target_link_libraries(flight_emu PUBLIC Eigen3::Eigen)
target_include_directories(flight_emu PUBLIC lib/FifoBuffer lib/EigenArduino lib/emulation lib/teensyShared lib/ServoControl)
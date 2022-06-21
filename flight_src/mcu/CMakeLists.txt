cmake_minimum_required(VERSION 3.10)

project(flight_emu_src)

file(GLOB lib_src lib/teensyShared/*.cpp lib/FifoBuffer/*.cpp lib/ServoControl/*.cpp)

add_library(flight_emu ${lib_src} lib/emulation/EMU.cpp src/mcu_ac/main.cpp)

target_link_libraries(flight_emu PUBLIC Eigen3::Eigen spdlog::spdlog CpuEmulation)

target_include_directories(flight_emu PUBLIC
        lib/FifoBuffer
        lib/EigenArduino
        lib/emulation
        lib/teensyShared
        lib/ServoControl
        ${SRC}/CpuEmulation
        # ${SRC}/Atmosphere
        ${SRC}/Sensors
        ${SRC}/Rocket
        ${SRC}/Aero
        ${SRC}/ControlSurfaces)

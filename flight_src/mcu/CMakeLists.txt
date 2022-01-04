cmake_minimum_required(VERSION 3.10)

project(flight_emu_src)

file(GLOB lib_src lib/teensyShared/*.c lib/teensyShared/*.cpp)

add_library(flight_emu ${lib_src} lib/emulation/EMU.cpp lib/emulation/ChRt.cpp)

target_include_directories(flight_emu PRIVATE lib/FifoBuffer lib/EigenArduino lib/emulation lib/teensyShared)
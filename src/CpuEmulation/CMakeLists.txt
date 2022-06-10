
add_library(CpuEmulation
            CpuState.h 
            CpuState.cpp
            CpuThread.h
            CpuThread.cpp
            CpuStateContext.h
            CpuStateContext.cpp)

target_include_directories(CpuEmulation PUBLIC
    ${SRC}/Sensors
    ${SRC}/ControlSurfaces
    ../../flight_src/mcu/lib/emulation
    ../../flight_src/mcu/lib/FifoBuffer
    ../../flight_src/mcu/lib/teensyShared)

target_link_libraries(CpuEmulation PUBLIC Sensors ControlSurfaces flight_emu)

target_link_libraries(CpuEmulation PRIVATE spdlog::spdlog Eigen3::Eigen)

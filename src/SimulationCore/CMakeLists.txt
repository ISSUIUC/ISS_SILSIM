
add_library(SimulationCore
            Simulation.h
            Simulation.cpp)

target_include_directories(SimulationCore PUBLIC
                           ${SRC}/Rocket
                           ${SRC}/Physics
                           ${SRC}/Aero
                           ${SRC}/Sensors
                           ${SRC}/CpuEmulation)

target_link_libraries(SimulationCore PUBLIC Rocket Physics Aero CpuEmulation)

target_link_libraries(SimulationCore PRIVATE spdlog::spdlog Eigen3::Eigen)

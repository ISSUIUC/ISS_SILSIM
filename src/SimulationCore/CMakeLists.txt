
add_library(SimulationCore
            Simulation.h
            Simulation.cpp
            Rocket.h
            Rocket.cpp)

target_include_directories(SimulationCore PUBLIC
                           ${SRC}/Physics
                           ${SRC}/Aero
                           ${SRC}/Sensors
                           ${SRC}/CpuEmulation)

target_link_libraries(SimulationCore PUBLIC Physics Aero CpuEmulation)

target_link_libraries(SimulationCore PRIVATE spdlog::spdlog Eigen3::Eigen)

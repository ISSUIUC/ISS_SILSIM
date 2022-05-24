
add_library(Sensors
            Sensor.h
            Sensor.cpp)

target_link_libraries(Sensors PUBLIC SimulationCore)

target_link_libraries(Sensors PRIVATE spdlog::spdlog Eigen3::Eigen)

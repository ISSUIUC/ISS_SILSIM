
add_library(Sensors
            Sensor.h
            Sensor.cpp)

target_include_directories(Sensors PUBLIC
                           ${SRC}/Rocket
                           ${SRC}/Atmosphere)

target_link_libraries(Sensors PUBLIC Rocket Atmosphere)

target_link_libraries(Sensors PRIVATE spdlog::spdlog Eigen3::Eigen)

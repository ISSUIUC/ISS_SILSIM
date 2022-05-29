
add_library(Sensors
            Sensor.h
            Sensor.cpp)

target_include_directories(Sensors PUBLIC
                           ${SRC}/Rocket)

target_link_libraries(Sensors PUBLIC Rocket)

target_link_libraries(Sensors PRIVATE spdlog::spdlog Eigen3::Eigen)

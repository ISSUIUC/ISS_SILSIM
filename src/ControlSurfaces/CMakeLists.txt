
add_library(ControlSurfaces
            Flaps.h
            Flaps.cpp)

target_include_directories(ControlSurfaces PUBLIC
                           ${SRC}/Rocket)

target_link_libraries(ControlSurfaces PUBLIC Rocket)

target_link_libraries(ControlSurfaces PRIVATE spdlog::spdlog Eigen3::Eigen)

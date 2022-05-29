
add_library(Rocket
            Rocket.h
            Rocket.cpp)

target_include_directories(Rocket PUBLIC
                           ${SRC}/Aero)

target_link_libraries(Rocket PUBLIC Aero)

target_link_libraries(Rocket PRIVATE spdlog::spdlog Eigen3::Eigen)

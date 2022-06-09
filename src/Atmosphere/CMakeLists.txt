
add_library(Atmosphere
            Atmosphere.h
            Atmosphere.cpp)

target_link_libraries(Atmosphere PRIVATE spdlog::spdlog Eigen3::Eigen)

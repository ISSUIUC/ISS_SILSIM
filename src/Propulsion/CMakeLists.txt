
add_library(Propulsion
            Propulsion.h
            Propulsion.cpp)

target_link_libraries(Propulsion PRIVATE spdlog::spdlog Eigen3::Eigen rapidcsv)

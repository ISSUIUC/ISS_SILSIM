
add_library(Aero
            RASAeroImport.h
            RASAeroImport.cpp)

target_link_libraries(Aero PRIVATE spdlog::spdlog Eigen3::Eigen rapidcsv)

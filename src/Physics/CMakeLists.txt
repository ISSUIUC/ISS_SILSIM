
add_library(Physics
            PhysicsEngine.h
            PhysicsEngine.cpp)

target_include_directories(Physics PUBLIC 
                           ${SRC}/SimulationCore
                           ${SRC}/Atmosphere
                           ${SRC}/Propulsion)

target_link_libraries(Physics PUBLIC SimulationCore Propulsion)

target_link_libraries(Physics PRIVATE spdlog::spdlog Eigen3::Eigen)

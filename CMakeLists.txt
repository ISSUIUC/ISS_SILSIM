cmake_minimum_required(VERSION 3.10)

project(SILSIM CXX)

set(CMAKE_CXX_STANDARD 17)


set(LIB ${CMAKE_SOURCE_DIR}/lib)
add_subdirectory(${LIB}/spdlog)



add_executable(ISS_SILSIM src/main.cpp
        src/CpuThread.cpp
        src/CpuState.cpp
        src/PhysicsEngine.cpp
        src/Propulsion.cpp
        src/Rocket.cpp
        src/Sensor.cpp
        src/Simulation.cpp
        lib/quaternion/quaternion.cpp
        lib/Vector3/Vector3.cpp
        )

#Linking raylib library
add_executable(visualizer sim_data/draw_rocket.cpp) #Change file to whatever is needed
add_subdirectory(lib/rapidcsv)
add_subdirectory(lib/raylib)
target_link_libraries(visualizer raylib rapidcsv)
target_link_libraries(ISS_SILSIM PRIVATE spdlog::spdlog)


#set highest warning levels
if(MSVC)
    target_compile_options(ISS_SILSIM PRIVATE /W3)
else()
    target_compile_options(ISS_SILSIM PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(ISS_SILSIM PUBLIC ${LIB}/quaternion ${LIB}/Vector3 ${CMAKE_SOURCE_DIR}/includes/)

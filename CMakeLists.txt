cmake_minimum_required(VERSION 3.10)

project(SILSIM)

set(CMAKE_CXX_STANDARD 17)

set(LIB ${CMAKE_SOURCE_DIR}/lib)

add_subdirectory(flight_src/mcu)

set(sources
        src/PhysicsEngine.cpp
        src/Propulsion.cpp
        src/Rocket.cpp
        src/Sensor.cpp
        src/Simulation.cpp
        src/Simulation.h
        src/CpuThread.h
        src/CpuThread.cpp
        src/CpuState.h
        src/CpuState.cpp
        )


#set highest warning levels
if(MSVC)
    target_compile_options(ISS_SILSIM PRIVATE /W3)
else()
    target_compile_options(ISS_SILSIM PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(ISS_SILSIM PUBLIC ${LIB}/quaternion ${LIB}/Vector3 ${CMAKE_SOURCE_DIR}/includes/)

add_executable(ISS_SILSIM src/main.cpp ${sources})

target_link_libraries(ISS_SILSIM flight_emu)

add_executable(CPU_STATE_TESTING src/cpu_state_testing_main.cpp ${sources})

target_include_directories(CPU_STATE_TESTING PUBLIC ${CMAKE_SOURCE_DIR}/lib/quaternion ${CMAKE_SOURCE_DIR}/lib/Vector3)

target_link_libraries(CPU_STATE_TESTING flight_emu)

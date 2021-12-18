cmake_minimum_required(VERSION 3.10)

project(SILSIM)


set(LIB ${CMAKE_SOURCE_DIR}/lib)

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

target_include_directories(ISS_SILSIM PUBLIC ${LIB}/quaternion ${LIB}/Vector3 ${CMAKE_SOURCE_DIR}/includes/)
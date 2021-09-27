cmake_minimum_required(VERSION 3.10)

project(SILSIM)

add_executable(ISS_SILSIM src/main.cpp
        src/PhysicsEngine.cpp
        src/PhysicsEngine.h
        src/Propulsion.cpp
        src/Propulsion.h
        src/Rocket.h
        src/Rocket.cpp
        src/Sensor.cpp
        src/Sensor.h
        src/Simulation.cpp
        src/Simulation.h
        lib/quaternion/quaternion.cpp
        lib/quaternion/quaternion.h
        lib/Vector3/Vector3.cpp
        lib/Vector3/Vector3.h)


target_include_directories(ISS_SILSIM PUBLIC ${CMAKE_SOURCE_DIR}/lib/quaternion ${CMAKE_SOURCE_DIR}/lib/Vector3)
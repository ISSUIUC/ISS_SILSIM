cmake_minimum_required(VERSION 3.10)

project(SILSIM)
set(CMAKE_CXX_STANDARD 17)
set(EIGEN_BUILD_DOC OFF)

set(LIB ${CMAKE_SOURCE_DIR}/lib)

if(
        NOT EXISTS ${LIB}/spdlog/CMakeLists.txt
        OR NOT EXISTS ${LIB}/eigen/CMakeLists.txt
)
    message(FATAL_ERROR "Missing submodule dependencies try\ngit submodule update --init --recursive\nto download dependencies")
endif()

add_subdirectory(${LIB}/spdlog)

add_subdirectory(flight_src/mcu)

add_executable(ISS_SILSIM src/main.cpp
        src/CpuThread.cpp
        src/CpuState.cpp
        src/PhysicsEngine.cpp
        src/Propulsion.cpp
        src/Rocket.cpp
        src/Sensor.cpp
        src/Simulation.cpp
        src/Atmosphere.cpp
        src/CpuStateContext.cpp)


#set highest warning levels
if(MSVC)
    target_compile_options(ISS_SILSIM PRIVATE /W3)
else()
    target_compile_options(ISS_SILSIM PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(ISS_SILSIM PRIVATE spdlog::spdlog flight_emu)
target_include_directories(ISS_SILSIM PUBLIC ${CMAKE_SOURCE_DIR}/includes/)

cmake_minimum_required(VERSION 3.10)

project(SILSIM)
set(CMAKE_CXX_STANDARD 17)
set(EIGEN_BUILD_DOC OFF)

set(LIB ${CMAKE_SOURCE_DIR}/lib)

#add_subdirectory(${LIB}/spdlog)
#add_subdirectory(${LIB}/eigen)
add_subdirectory(${LIB}/rapidcsv)

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
        src/CpuStateContext.cpp
        src/RASAeroImport.cpp
        )

#set highest warning levels
if(MSVC)
    target_compile_options(ISS_SILSIM PRIVATE /W3)
else()
#    -Wno-unknown-pragmas to silence eigen pragma warnings
    target_compile_options(ISS_SILSIM PRIVATE -Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

target_link_libraries(ISS_SILSIM PRIVATE rapidcsv flight_emu)
target_include_directories(ISS_SILSIM PUBLIC ${CMAKE_SOURCE_DIR}/includes/)

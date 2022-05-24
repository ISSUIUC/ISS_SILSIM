cmake_minimum_required(VERSION 3.10)

project(SILSIM)
set(CMAKE_CXX_STANDARD 17)


set(SRC ${CMAKE_SOURCE_DIR}/src)
set(LIB ${CMAKE_SOURCE_DIR}/lib)

if(NOT EXISTS ${LIB}/spdlog/CMakeLists.txt
   OR NOT EXISTS ${LIB}/eigen/CMakeLists.txt)
    message(FATAL_ERROR "Missing submodule dependencies try\ngit submodule update --init --recursive\nto download dependencies")
endif()

# Add SILSIM source code directories
add_subdirectory(${SRC}/SimulationCore)
add_subdirectory(${SRC}/Physics)
add_subdirectory(${SRC}/Propulsion)
add_subdirectory(${SRC}/Atmosphere)
add_subdirectory(${SRC}/Aero)
add_subdirectory(${SRC}/Sensors)
add_subdirectory(${SRC}/CpuEmulation)

# Add external libraries
add_subdirectory(${LIB}/spdlog)
add_subdirectory(${LIB}/eigen)
add_subdirectory(${LIB}/rapidcsv)


add_executable(ISS_SILSIM src/main.cpp)

# Link SILSIM source code to main executable
target_link_libraries(ISS_SILSIM PUBLIC 
                      SimulationCore
                      Physics
                      Propulsion
                      Atmosphere
                      Aero
                      Sensors
                      CpuEmulation)

# Link external libraries to main executable
target_link_libraries(ISS_SILSIM PRIVATE 
                      spdlog::spdlog 
                      Eigen3::Eigen 
                      rapidcsv)

target_include_directories(ISS_SILSIM PUBLIC 
                           ${SRC}/SimulationCore
                           ${SRC}/Physics
                           ${SRC}/Propulsion
                           ${SRC}/Atmosphere
                           ${SRC}/Aero
                           ${SRC}/Sensors
                           ${SRC}/CpuEmulation)

#set highest warning levels
if(MSVC)
    target_compile_options(ISS_SILSIM PRIVATE /W3)
else()
    target_compile_options(ISS_SILSIM PRIVATE -Wall -Wextra -Wpedantic)
endif()


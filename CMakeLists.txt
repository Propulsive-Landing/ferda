cmake_minimum_required(VERSION 3.5)

project(Ferda C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

# Add options for the Simulation build type and parameters
option(BUILD_SIMULATION "Build for simulation" OFF)
set(SIM_LOCAL_PORT "8002" CACHE STRING "Local port for simulation")
set(SIM_SERVER_PORT "8003" CACHE STRING "Server port for simulation")
set(SIM_SERVER_IP "127.0.0.1" CACHE STRING "Server IP for simulation")

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    FILE(GLOB EXTRA_CODE hardware_test/*.cpp)
elseif(CMAKE_BUILD_TYPE STREQUAL Simulation)
    FILE(GLOB EXTRA_CODE hardware_simulation/*.cpp)
else()
    FILE(GLOB EXTRA_CODE hardware/*.cpp)
endif()

FILE(GLOB CppSources src/*.cpp)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(Ferda ${CppSources} ${EXTRA_CODE})

target_include_directories(Ferda PRIVATE include)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_libraries(Ferda Eigen3::Eigen)
elseif(CMAKE_BUILD_TYPE STREQUAL Simulation)
    target_link_libraries(Ferda Eigen3::Eigen)
    target_compile_definitions(Ferda PRIVATE 
        SIMULATION_MODE
        SIM_LOCAL_PORT=${SIM_LOCAL_PORT}
        SIM_SERVER_PORT=${SIM_SERVER_PORT}
        SIM_SERVER_IP="${SIM_SERVER_IP}"
    )
else()
    target_link_libraries(Ferda Eigen3::Eigen pigpio)
endif()

# Add threading library
find_package(Threads REQUIRED)
target_link_libraries(Ferda Threads::Threads)
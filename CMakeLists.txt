cmake_minimum_required(VERSION 3.5)

project(Ferda C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    set(EXTRA_CODE hardware_test/Barometer.cpp)
else()
    set(EXTRA_CODE hardware/Barometer.cpp)
endif()

FILE(GLOB CppSources src/*.cpp)

add_executable(Ferda ${CppSources} ${EXTRA_CODE})
target_include_directories(Ferda PRIVATE include)

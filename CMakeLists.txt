cmake_minimum_required(VERSION 3.5)

project(Ferda C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    set(EXTRA_CODE hardware_test/Barometer.cpp)
else()
    set(EXTRA_CODE hardware/Barometer.cpp)
endif()

add_executable(Ferda src/Main.cpp src/Controller.cpp ${EXTRA_CODE})
target_include_directories(Ferda PRIVATE include)

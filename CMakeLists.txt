cmake_minimum_required(VERSION 3.5)

project(Ferda C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    FILE(GLOB EXTRA_CODE hardware_test/*.cpp)
else()
    FILE(GLOB EXTRA_CODE hardware/*.cpp)
endif()

FILE(GLOB CppSources src/*.cpp)

find_package (Eigen3 REQUIRED NO_MODULE)

add_executable(Ferda ${CppSources} ${EXTRA_CODE})
target_include_directories(Ferda PRIVATE include hardware_test/simulation/sims)

target_link_libraries (Ferda Eigen3::Eigen -L"/workspaces/ferda/hardware_test/simulation/build")
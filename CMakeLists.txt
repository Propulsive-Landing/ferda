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

find_package(Eigen3 REQUIRED NO_MODULE)

# Fetch GoogleTest content
include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(Ferda ${CppSources} ${EXTRA_CODE})

target_include_directories(Ferda PRIVATE include)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_libraries(Ferda Eigen3::Eigen gtest_main)
    # Link with Google Test and Google Test Main
    target_link_libraries(Ferda gtest gtest_main)
else()
    target_link_libraries(Ferda Eigen3::Eigen pigpio)
endif()


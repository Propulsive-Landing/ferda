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




MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list *.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()
header_directories(model_includes)


add_executable(Ferda ${CppSources} ${EXTRA_CODE})

target_include_directories(Ferda PRIVATE include ${model_includes})
target_link_directories(Ferda PRIVATE hardware_test/simulation/build)
target_link_libraries (Ferda Eigen3::Eigen sim)
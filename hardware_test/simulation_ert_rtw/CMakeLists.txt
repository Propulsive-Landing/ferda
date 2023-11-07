###########################################################################
# CMakeLists.txt generated for component simulation
# Product type: executable
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(simulation)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the 
# environment if it is defined as an environment variable, but not as a 
# CMake variable. This is to work around a bug in CMake 3.19 when the 
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

# Define common variables that are used within the whole project.
set(SYSLIB_PREFIX $<IF:$<AND:$<BOOL:${WIN32}>,$<NOT:$<BOOL:${MINGW}>>>,lib,>)

###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR .. ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
set(MATLAB_ROOT /MATLAB CACHE PATH "")

# Additional variables that are defined conditionally.
if("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

if(NOT TARGET ssc_sli)
    # Definition of target "ssc_sli".
    add_library(ssc_sli STATIC ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_0267402d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_0763c151.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_0bd269e6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_0d6d0780.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_136b443c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_136f97a1.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_140b3534.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_15d5f6ce.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_1c690f90.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_1db813e8.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_256cd4b6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_2bbd58a4.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_2f6ea1cd.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_360cfd63.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_37d957ac.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_3fb4607e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_40d56d93.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_42b14021.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_43618287.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_466b08dd.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_496799bd.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_4e028390.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_550a4805.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_5a0cb974.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_5d63aeeb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_62d81790.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_6305062f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_77063d8b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_7a618260.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_7f65db03.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_880e593a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_89d0f30a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_8a64c4e2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_93019ea6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_9abcdb7f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_9b67747c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_9c030181.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_c7dda239.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_d064c978.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_d80c44d2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_dcd66f69.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_e66fe6d5.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_e7b327bb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_eb0a5702.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_edbee55c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_f6bd9cc8.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_f9b5dbc5.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_fa0ce53e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/src/ssc_sli_fbdf29da.c)
    
    # Set properties for target "ssc_sli".
    set_target_properties(ssc_sli PROPERTIES PREFIX ""
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>")
    
    # Specify language features required for target "ssc_sli".
    target_compile_features(ssc_sli PUBLIC cxx_std_11)
    
    # Specify compiler preprocessor definitions for target "ssc_sli".
    target_compile_definitions(ssc_sli PRIVATE ${SIMULATION_COMMON_DEFINES})
    
    # Specify include directories for target "ssc_sli".
    target_include_directories(ssc_sli PUBLIC $<BUILD_INTERFACE:${START_DIR}>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
        $<BUILD_INTERFACE:${START_DIR}/simulation_ert_rtw>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/simulation_ert_rtw>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include>)
    
    # Specify library link dependencies for target "ssc_sli".  CMake 
    # generator expressions are used to create a CMakeLists.txt file that 
    # supports multiple platforms with differently named system library 
    # dependencies.
    target_link_libraries(ssc_sli INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)
endif()


if(NOT TARGET sm_ssci)
    # Definition of target "sm_ssci".
    add_library(sm_ssci STATIC ${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/src/sm_ssci_3dd14f0a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/src/sm_ssci_646478c5.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/src/sm_ssci_b2b6b422.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/src/sm_ssci_c16a187b.c)
    
    # Set properties for target "sm_ssci".
    set_target_properties(sm_ssci PROPERTIES PREFIX ""
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>")
    
    # Specify language features required for target "sm_ssci".
    target_compile_features(sm_ssci PUBLIC cxx_std_11)
    
    # Specify compiler preprocessor definitions for target "sm_ssci".
    target_compile_definitions(sm_ssci PRIVATE ${SIMULATION_COMMON_DEFINES})
    
    # Specify include directories for target "sm_ssci".
    target_include_directories(sm_ssci PUBLIC $<BUILD_INTERFACE:${START_DIR}>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
        $<BUILD_INTERFACE:${START_DIR}/simulation_ert_rtw>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/simulation_ert_rtw>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include>)
    
    # Specify library link dependencies for target "sm_ssci".  CMake 
    # generator expressions are used to create a CMakeLists.txt file that 
    # supports multiple platforms with differently named system library 
    # dependencies.
    target_link_libraries(sm_ssci INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)
endif()


if(NOT TARGET ssc_core)
    # Definition of target "ssc_core".
    add_library(ssc_core STATIC ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_01dcc633.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_04da2c69.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_05058dd9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_06ba68a6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_09b5fa6e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_0bd666aa.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_0d65be6c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_0f019bd9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_0f0420a6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_18bf4d77.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_1c6b0332.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_1fd25120.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_24b4cdee.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_280c0222.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_2cd54448.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_3169e4b7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_32d8b307.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_3306c008.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_360a4baf.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_37d4ea84.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_40db642d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_40dfdbdc.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_41017299.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_4666b45b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_48b1386a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_4ad9135b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_4db6bd68.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_4db86fcc.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_4e03e39d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_4e04eecd.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_4e06e3bd.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_54d55ae9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_5505224d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_550b4c41.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_56b1a2bf.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_576cd129.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_59b034b8.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_5a046b27.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_5d6ba758.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_67d1f118.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_68da074b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_6b663a17.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_6b6b89d2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_6dd833f3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_7209d3a5.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_73d9c2b7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_76d825be.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_770cead7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_7a613edb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_83db8762.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_856738f2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_8569edc5.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_89d7fa79.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_8a6471dc.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_8cd9cb37.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_8d0064b8.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_96061071.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_990fe1a4.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_9b607b15.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_9b671e57.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_9c016445.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_9c01d168.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_9dd110ad.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_9fb0e229.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_9fb25b4f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_9fb2e56c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a1d393be.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a269ab09.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a4d4c45e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a4da1d0a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a6b78ccc.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a76299bc.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a867d880.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a9bbaecc.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_a9bf1ff2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_aa09f23c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_abd05c18.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_abd5e7b4.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_acb64294.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_acb6462e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_b1038cbb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_b10e34f4.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_b2b3b239.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_b369cd13.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_b402b40d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_b7b88213.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_b96ebc21.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_bad68669.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_bc648043.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_bfdb08db.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_c3003040.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_c5b050d7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_c5b63cb2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_c607b660.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_c8d83e88.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_cab615c8.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_cabdc251.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_cc067f58.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_cc0bcab0.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_ce6a84bb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_d06d763c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_d3d34d7c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_d4b9397b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_d70a6a09.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_d807fa59.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_dcda6edd.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_deb7fd8d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_e0d0866d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_e2b61d72.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_e400c1c2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_ead8f455.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_edb836de.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_ee000fbe.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_ee01086d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_ee0f0141.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_f9b6dbed.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_fa09e9e6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_fbd34e62.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/src/ssc_core_ff06d9a4.c)
    
    # Set properties for target "ssc_core".
    set_target_properties(ssc_core PROPERTIES PREFIX ""
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>")
    
    # Specify language features required for target "ssc_core".
    target_compile_features(ssc_core PUBLIC cxx_std_11)
    
    # Specify compiler preprocessor definitions for target "ssc_core".
    target_compile_definitions(ssc_core PRIVATE ${SIMULATION_COMMON_DEFINES})
    
    # Specify include directories for target "ssc_core".
    target_include_directories(ssc_core PUBLIC $<BUILD_INTERFACE:${START_DIR}>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
        $<BUILD_INTERFACE:${START_DIR}/simulation_ert_rtw>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/simulation_ert_rtw>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include>)
    
    # Specify library link dependencies for target "ssc_core".  CMake 
    # generator expressions are used to create a CMakeLists.txt file that 
    # supports multiple platforms with differently named system library 
    # dependencies.
    target_link_libraries(ssc_core INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)
endif()


if(NOT TARGET ssc_st)
    # Definition of target "ssc_st".
    add_library(ssc_st STATIC ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_12be8102.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_43603aca.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_59ba8c5e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_65b906f8.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_7f64b2fa.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_870f70cf.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_a3b260e9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_a7639c45.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_a868d115.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/src/ssc_st_d56af4a2.c)
    
    # Set properties for target "ssc_st".
    set_target_properties(ssc_st PROPERTIES PREFIX ""
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>")
    
    # Specify language features required for target "ssc_st".
    target_compile_features(ssc_st PUBLIC cxx_std_11)
    
    # Specify compiler preprocessor definitions for target "ssc_st".
    target_compile_definitions(ssc_st PRIVATE ${SIMULATION_COMMON_DEFINES})
    
    # Specify include directories for target "ssc_st".
    target_include_directories(ssc_st PUBLIC $<BUILD_INTERFACE:${START_DIR}>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
        $<BUILD_INTERFACE:${START_DIR}/simulation_ert_rtw>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/simulation_ert_rtw>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include>)
    
    # Specify library link dependencies for target "ssc_st".  CMake 
    # generator expressions are used to create a CMakeLists.txt file that 
    # supports multiple platforms with differently named system library 
    # dependencies.
    target_link_libraries(ssc_st INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)
endif()


if(NOT TARGET mc)
    # Definition of target "mc".
    add_library(mc STATIC ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_026e4f4b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_03b98f6f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_0764157d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_0bd30dee.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_0d6fd085.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_0ed73c49.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_0edc34e3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_10d4ab75.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_11086079.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_110e6c6c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_1407e917.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_140e3c4c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_15d12d95.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_15d828ca.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_220ba961.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_2565d6e0.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_2a642f54.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_2a6a9b24.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_2a6b417d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_2bbf87e3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_2cdc96b4.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_32d501e3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_32dc008a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_3b6a945d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_3e66abdf.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_4105189f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_47b8cebe.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_47b91db1.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_4b0301c6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_4c6117e3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_51d4094e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_52623861.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_52688a58.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_53b1fc84.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_53b6fcf3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_550847c3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_5766048f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_59b6e413.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_5bd2c9c1.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_5d65cd86.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_5edd42ef.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_630208f8.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_630dda0e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_67da200d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_67da4f41.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_6b6d311a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_6e61d16c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_6fb1c336.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7207b6f4.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7809a65c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7a613aec.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7bbf41f0.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7cd58f0b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7cdbe436.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7d0547c8.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7d099de7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_7eb21b39.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_81b0ada5.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_81b5717e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_84b5f9fb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_870ec75e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_89d597cf.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_90b6aa0a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_95b62b73.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_9ab7d9b0.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_9b6376d1.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_9b6c1529.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_a00e5f46.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_a2647600.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_a26bab1a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_a3b90582.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_a6b5ed24.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_a7684938.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_a865d1dd.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_acb3fad7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_af0bc203.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_af0cc4c9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_b0de9cbc.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_b2b86ae3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_b362c5eb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_b7b03d44.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_b96a0bad.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_bb0520ee.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_bdbb9b78.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_c2dbf4b2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_c7de7352.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_c8d25d23.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_cab8a1f9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_ce6656ce.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_d20085b7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_d9d38185.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_dbbb14d2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_dcdddfae.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_debb448f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_e7b99cf6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_e7bc2f1a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_e9661c4a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_e969ae87.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_edbf543c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_ee000fbe.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_efdea3a7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_f0012953.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_f00d2f9b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_f3be157c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_fbd54145.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_fcb15a9b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_fd619d14.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/src/mc_fd6341bb.c)
    
    # Set properties for target "mc".
    set_target_properties(mc PROPERTIES PREFIX ""
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>")
    
    # Specify language features required for target "mc".
    target_compile_features(mc PUBLIC cxx_std_11)
    
    # Specify compiler preprocessor definitions for target "mc".
    target_compile_definitions(mc PRIVATE ${SIMULATION_COMMON_DEFINES})
    
    # Specify include directories for target "mc".
    target_include_directories(mc PUBLIC $<BUILD_INTERFACE:${START_DIR}>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
        $<BUILD_INTERFACE:${START_DIR}/simulation_ert_rtw>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/simulation_ert_rtw>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include>)
    
    # Specify library link dependencies for target "mc".  CMake generator 
    # expressions are used to create a CMakeLists.txt file that supports 
    # multiple platforms with differently named system library 
    # dependencies.
    target_link_libraries(mc INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)
endif()


if(NOT TARGET sm)
    # Definition of target "sm".
    add_library(sm STATIC ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_01d3ac26.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_04d045b9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_09b59221.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_0cb6a5fa.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_1dba7276.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_1e022f29.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_26df822b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_38d4abbe.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_43618472.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_440126a7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_45dc522c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_466cdd79.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_4b066b5a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_4c61739a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_5763bdb7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_6166f1eb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_62d41fb5.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_630c0817.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_67d72683.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_6fbd150d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_73d210b9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_916806b3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_95b82517.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_9abcb56e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_a0028316.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_a1d32671.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_a3ba6de7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_a50cd7b1.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_aa059a3a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_acba2496.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_b40db855.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_b66990f5.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_badd8656.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_bc63e36c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_bfda6e42.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_c0ba649d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_c5b9546f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_c60bdc8d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_c7d3720c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_cabca465.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_d3d79f0f.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_d3d946fd.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_d4bb3938.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_d6dc1822.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_da690167.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_dbbace78.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_df66e89b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_e8bab6d7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_ec6029c9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_efdfa66e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/src/sm_f3b314ce.c)
    
    # Set properties for target "sm".
    set_target_properties(sm PROPERTIES PREFIX ""
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>")
    
    # Specify language features required for target "sm".
    target_compile_features(sm PUBLIC cxx_std_11)
    
    # Specify compiler preprocessor definitions for target "sm".
    target_compile_definitions(sm PRIVATE ${SIMULATION_COMMON_DEFINES})
    
    # Specify include directories for target "sm".
    target_include_directories(sm PUBLIC $<BUILD_INTERFACE:${START_DIR}>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
        $<BUILD_INTERFACE:${START_DIR}/simulation_ert_rtw>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/simulation_ert_rtw>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include>)
    
    # Specify library link dependencies for target "sm".  CMake generator 
    # expressions are used to create a CMakeLists.txt file that supports 
    # multiple platforms with differently named system library 
    # dependencies.
    target_link_libraries(sm INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)
endif()


if(NOT TARGET pm_math)
    # Definition of target "pm_math".
    add_library(pm_math STATIC ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_01dec08e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_04d19358.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_050e87e9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_1966ea7d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_1ad202b7.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_1c69d5b2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_1c6b6a1b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_29dc7f63.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_2a66252c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_2f607a02.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_32d9636d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_3309726a.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_3463da5d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_360e4b46.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_3fbf046b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_48b75553.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_48bd51fb.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_5a01dda4.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_646fa971.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_6b69e06e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_7565727c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_7cd5543c.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_89df260e.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_8cd61cd6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_8d05b7c0.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_93014cb0.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_a001e9ec.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_b7b980b1.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_bad43c87.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_c7d2cd09.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_d1be0f30.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_d80b9e3b.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_da630bd2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_e8b06b42.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_ee02b2b3.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_f760e8f6.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_f866cdd2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/src/pm_math_fd679a52.c)
    
    # Set properties for target "pm_math".
    set_target_properties(pm_math PROPERTIES PREFIX ""
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>")
    
    # Specify language features required for target "pm_math".
    target_compile_features(pm_math PUBLIC cxx_std_11)
    
    # Specify compiler preprocessor definitions for target "pm_math".
    target_compile_definitions(pm_math PRIVATE ${SIMULATION_COMMON_DEFINES})
    
    # Specify include directories for target "pm_math".
    target_include_directories(pm_math PUBLIC $<BUILD_INTERFACE:${START_DIR}>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
        $<BUILD_INTERFACE:${START_DIR}/simulation_ert_rtw>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/simulation_ert_rtw>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include>)
    
    # Specify library link dependencies for target "pm_math".  CMake 
    # generator expressions are used to create a CMakeLists.txt file that 
    # supports multiple platforms with differently named system library 
    # dependencies.
    target_link_libraries(pm_math INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)
endif()


if(NOT TARGET pm)
    # Definition of target "pm".
    add_library(pm STATIC ${MATLAB_ROOT}/extern/physmod/glnxa64/pm/src/pm_09bc42e2.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm/src/pm_14098e54.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm/src/pm_26dc3230.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm/src/pm_4fd5f5b9.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm/src/pm_9dd7a82d.c
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm/src/pm_fed8c2c9.c)
    
    # Set properties for target "pm".
    set_target_properties(pm PROPERTIES PREFIX ""
        POSITION_INDEPENDENT_CODE ON
        RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>"
        ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/simulation_ert_rtw/$<0:>")
    
    # Specify language features required for target "pm".
    target_compile_features(pm PUBLIC cxx_std_11)
    
    # Specify compiler preprocessor definitions for target "pm".
    target_compile_definitions(pm PRIVATE ${SIMULATION_COMMON_DEFINES})
    
    # Specify include directories for target "pm".
    target_include_directories(pm PUBLIC $<BUILD_INTERFACE:${START_DIR}>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
        $<BUILD_INTERFACE:${START_DIR}/simulation_ert_rtw>
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/simulation_ert_rtw>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/simulink/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/src/ext_mode/common>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/rtw/c/ert>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include>
        $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include>)
    
    # Specify library link dependencies for target "pm".  CMake generator 
    # expressions are used to create a CMakeLists.txt file that supports 
    # multiple platforms with differently named system library 
    # dependencies.
    target_link_libraries(pm INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)
endif()


# When a CMakeLists.txt defines multiple targets that have a common set of 
# tool flags and preprocessor definitions, define variables containing 
# data that is common to all targets defined in the file.
set(SIMULATION_COMMON_DEFINES -DMODEL=simulation
    -DNUMST=3
    -DNCSTATES=30
    -DHAVESTDIO
    -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
    -DCLASSIC_INTERFACE=0
    -DALLOCATIONFCN=0
    -DTID01EQ=1
    -DTERMFCN=1
    -DONESTEPFCN=1
    -DMAT_FILE=0
    -DMULTI_INSTANCE_CODE=0
    -DINTEGER_CODE=0
    -DMT=0)

# Definition of target "simulation".
add_executable(simulation  ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_create.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_setParameters.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_asserts.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_deriv.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_checkDynamics.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_compOutputsKin.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_compOutputsDyn.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_asm_delegate.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_sim_delegate.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_mode_zero_crossings.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_logging.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_geometries.c
    ${START_DIR}/simulation_ert_rtw/simulation_b048d748_1_gateway.c
    ${START_DIR}/simulation_ert_rtw/rt_backsubrr_dbl.c
    ${START_DIR}/simulation_ert_rtw/rt_forwardsubrr_dbl.c
    ${START_DIR}/simulation_ert_rtw/rt_lu_real.c
    ${START_DIR}/simulation_ert_rtw/rt_matrixlib_dbl.c
    ${START_DIR}/simulation_ert_rtw/ert_main.cpp
    ${START_DIR}/simulation_ert_rtw/rtGetInf.cpp
    ${START_DIR}/simulation_ert_rtw/rtGetNaN.cpp
    ${START_DIR}/simulation_ert_rtw/rt_nonfinite.cpp
    ${START_DIR}/simulation_ert_rtw/simulation.cpp
    ${START_DIR}/simulation_ert_rtw/simulation_data.cpp)

# Set properties for target "simulation".
set_target_properties(simulation PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>")

# Specify language features required for target "simulation".
target_compile_features(simulation PUBLIC cxx_std_11)

# Specify compiler preprocessor definitions for target "simulation".
target_compile_definitions(simulation PRIVATE ${SIMULATION_COMMON_DEFINES})

# Specify include directories for target "simulation".
target_include_directories(simulation PRIVATE ${START_DIR}
    ${START_DIR}/simulation_ert_rtw
    ${MATLAB_ROOT}/extern/include
    ${MATLAB_ROOT}/simulink/include
    ${MATLAB_ROOT}/rtw/c/src
    ${MATLAB_ROOT}/rtw/c/src/ext_mode/common
    ${MATLAB_ROOT}/rtw/c/ert
    ${MATLAB_ROOT}/extern/physmod/glnxa64/lang/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_comp/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include
    ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_st/include)

# Specify library link dependencies for target "simulation".  CMake 
# generator expressions are used to create a CMakeLists.txt file that 
# supports multiple platforms with differently named system library 
# dependencies.
target_link_libraries(simulation PRIVATE ssc_sli
    sm_ssci
    ssc_core
    ssc_st
    mc
    sm
    pm_math
    pm
    $<$<BOOL:${FOUND_LIBM}>:m>
    $<$<BOOL:${WIN32}>:kernel32>
    $<$<BOOL:${WIN32}>:ws2_32>
    $<$<BOOL:${WIN32}>:mswsock>
    $<$<BOOL:${WIN32}>:advapi32>)

# Extract DWARF debug symbols into a separate file when building 
# executable programs for Apple platforms.
if(APPLE AND (CMAKE_C_COMPILER_ID STREQUAL "AppleClang"))
    string(APPEND _dsymutilcmd "$<IF:$<CONFIG:DEBUG,RELWITHDEBINFO>,"
                "xcrun;dsymutil;$<TARGET_FILE:simulation>;--flat,"
                "true"
        >)
    add_custom_command(TARGET simulation
            POST_BUILD
            COMMAND "${_dsymutilcmd}"
            COMMAND_EXPAND_LISTS
            BYPRODUCTS simulation.dwarf)
endif()


###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET simulation POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created executable: $<TARGET_FILE:simulation>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(simulation)
endif()

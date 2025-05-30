﻿cmake_minimum_required(VERSION 3.16)
project(uasrisk
        DESCRIPTION "Risk calculation for Unmanned Aerial Systems"
        VERSION 0.1
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CXX_STANDARD_REQUIRED ON)
set(PYTHON_VERSION 3)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
Include(FetchContent)
include_guard(GLOBAL)

# Verbose Builds
# set(CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "ON" FORCE)

### Require out-of-source builds
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)
if (EXISTS "${LOC_PATH}")
    message(FATAL_ERROR "You cannot build in a source directory (or any directory with a CMakeLists.txt file). Please make a build subdirectory. Feel free to remove CMakeCache.txt and CMakeFiles.")
endif ()

# Cache builds
find_program(CCACHE ccache)
if (CCACHE)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
endif ()

option(UR_BUILD_PYTHON_BINDINGS "Build the Python bindings for uasrisk" ON)
option(UR_BUILD_TESTS "Build the tests for uasrisk" OFF)
option(BUILD_SHARED_LIBS "Build as shared library" ON)
option(UR_BUILD_CLI "Build the command line interface" ON)

# Enable all targets if using CLion otherwise everything breaks :/
if ($ENV{CLION_IDE})
    set(UR_BUILD_PYTHON_BINDINGS ON)
    set(UR_BUILD_TESTS ON)
    set(UR_BUILD_CLI ON)
endif ()

###########################################################
# Static code analysis
###########################################################
if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(ENABLE_CPPCHECK ON)
else()
    set(ENABLE_CPPCHECK OFF)
endif()
if (MSVC)
    set(ENABLE_CLANG_TIDY OFF)
else ()
    set(ENABLE_CLANG_TIDY ON)
endif ()

if (ENABLE_CPPCHECK)
    find_program(CPPCHECK cppcheck)
    if (CPPCHECK)
        set(CMAKE_CXX_CPPCHECK
                ${CPPCHECK}
                --suppressions-list=${CMAKE_CURRENT_SOURCE_DIR}/.cppcheck-suppressions
                --suppress=missingIncludeSystem
                --suppress=unmatchedSuppression
                --cppcheck-build-dir=${CMAKE_BINARY_DIR}/cppcheck
                --enable=all
                --inconclusive)
    else ()
        message(WARNING "cppcheck requested but executable not found")
    endif ()
endif ()

if (ENABLE_CLANG_TIDY)
    find_program(CLANGTIDY clang-tidy)
    if (CLANGTIDY)
        set(CMAKE_CXX_CLANG_TIDY ${CLANGTIDY})
    else ()
        message(WARNING "clang-tidy requested but executable not found")
    endif ()
endif ()



##############################################################
# Packaging + Linking
##############################################################

set(UR_SOURCES
        ${UR_SOURCES})

add_subdirectory(src)

if (MSVC)
    if(BUILD_SHARED_LIBS)
        message("Only static linking is supported for MSVC compilers")
    endif()
    set(BUILD_SHARED_LIBS OFF)
endif ()

add_library(${PROJECT_NAME} ${UR_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

target_include_directories(${PROJECT_NAME}
	PUBLIC
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>)

add_subdirectory(include)

if (MSVC)
    # Prevent Windows.h overwriting min and max with macros
    # Keep Math defines such as M_PI
    target_compile_definitions(${PROJECT_NAME} PUBLIC NOMINMAX _USE_MATH_DEFINES)
    target_compile_options(${PROJECT_NAME} PUBLIC /permissive-)
endif ()

##############################################################
# Dependencies
##############################################################
# Don't build tests for deps
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

# Eigen required for lin alg
find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

find_package(proj CONFIG REQUIRED)
# Get Data dir in conan package
string(TOUPPER ${CMAKE_BUILD_TYPE} ${PROJECT_NAME}_UPPER_BUILD_TYPE)
set(PROJ_DATA "${proj_PACKAGE_FOLDER_${${PROJECT_NAME}_UPPER_BUILD_TYPE}}/res")

# check if PROJ_DATA is empty
if (NOT DEFINED PROJ_DATA OR PROJ_DATA STREQUAL "")
    # Hacky way to get the data dir from the conan package
    message(WARNING "Could not find PROJ data dir for build type ${CMAKE_BUILD_TYPE} using proj_PACKAGE_FOLDER_${${PROJECT_NAME}_UPPER_BUILD_TYPE} config target variable")
    set(PROJ_DATA "${PROJ_INCLUDE_DIR_${${PROJECT_NAME}_UPPER_BUILD_TYPE}}/../res/")
else ()
    message(STATUS "Found PROJ data dir: ${PROJ_DATA}")
endif ()

# Check that proj.db actually exists in PROJ_DATA
if (NOT EXISTS "${PROJ_DATA}/proj.db")
    message(FATAL_ERROR "Could not find proj.db in assumed PROJ_DATA dir: ${PROJ_DATA}")
endif ()
add_custom_target(UR_proj_data
	${CMAKE_COMMAND} -E copy_directory ${PROJ_DATA} ${PROJECT_BINARY_DIR}/bin)
add_dependencies(${PROJECT_NAME} UR_proj_data)
# Set this to a compile def that is used when PROJ contexts are created
target_compile_definitions(${PROJECT_NAME} PUBLIC PROJ_DATA_PATH="${PROJ_DATA}")
target_link_libraries(${PROJECT_NAME} PUBLIC PROJ::proj)

find_package(OpenMP REQUIRED)
target_link_directories(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)

#find_package(netCDF CONFIG REQUIRED)
#target_link_libraries(${PROJECT_NAME} PRIVATE netCDF::netcdf)

find_package(Boost REQUIRED CONFIG)
target_link_libraries(${PROJECT_NAME} PUBLIC Boost::boost)

find_package(RapidJSON REQUIRED CONFIG)
target_link_libraries(${PROJECT_NAME} PRIVATE rapidjson)

find_package(fast-cpp-csv-parser REQUIRED CONFIG)

find_package(pugixml REQUIRED CONFIG)
target_link_libraries(${PROJECT_NAME} PRIVATE pugixml::pugixml)

find_package(spdlog CONFIG REQUIRED CONFIG)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)

# Set data dir for python bindings
if(UR_BUILD_PYTHON_BINDINGS)
    find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
    set(UGR_DATA_DIR "${Python_SITELIB}/pyuasrisk/data")
    message(STATUS "UGR_DATA_DIR: ${UGR_DATA_DIR}")
endif()

message(STATUS "BUILDING uasgroundrisk...")
FetchContent_Declare(
        uasgroundrisk
        GIT_REPOSITORY https://github.com/aliaksei135/uasgroundrisk.git
        GIT_TAG origin/master
)
FetchContent_MakeAvailable(uasgroundrisk)
target_link_libraries(${PROJECT_NAME} PUBLIC uasgroundrisk)

if(UR_BUILD_PYTHON_BINDINGS)
    message(STATUS "Building Python bindings...")
    add_subdirectory(extern/pybind11)
    add_subdirectory(python)
endif()

##############################################################
# Testing
##############################################################

if (UR_BUILD_TESTS)
    ##################################
    # GoogleTest
    # Test framework
    # License: BSD-3
    ##################################
    if (NOT TARGET gtest)
        FetchContent_Declare(
                googletest
                GIT_REPOSITORY https://github.com/google/googletest.git
                GIT_TAG release-1.17.0
        )
        # For Windows: Prevent overriding the parent project's compiler/linker settings
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif ()

    enable_testing()
    add_subdirectory(test)
endif ()

#############################################################
# Install
#############################################################
include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

#IF(UR_BUILD_CLI)
#    message(STATUS "Building CLI...")
#    add_subdirectory(cli)
#endif()


#############################################################
# Docs
#############################################################

if (BUILD_DOC)
    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        SET(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
        SET(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

        add_custom_target(doc ALL
                COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Building Doxygen documentation"
                VERBATIM)
    else (DOXYGEN_FOUND)
        message("No doxygen binary found on the system.")
        SET(${BUILD_DOC} OFF)
    endif ()
endif ()

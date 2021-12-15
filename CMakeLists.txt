cmake_minimum_required(VERSION 3.16)
project(uasrisk
        DESCRIPTION "Risk calculation for Unmanned Aerial Systems"
        VERSION 0.1
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
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

###########################################################
# Static code analysis
###########################################################
if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(ENABLE_CPPCHECK OFF)
else()
    set(ENABLE_CPPCHECK ON)
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
include_directories(include)

if (MSVC)
    set(${PROJECT_NAME}_BUILD_SHARED OFF)
else ()
    set(${PROJECT_NAME}_BUILD_SHARED ON)
endif ()

if (${${PROJECT_NAME}_BUILD_SHARED})
    add_library(${PROJECT_NAME} SHARED ${UR_SOURCES})
else ()
    add_library(${PROJECT_NAME} STATIC ${UR_SOURCES})
endif ()

if (MSVC)
    # Prevent Windows.h overwriting min and max with macros
    # Keep Math defines such as M_PI
    target_compile_definitions(${PROJECT_NAME} PUBLIC NOMINMAX _USE_MATH_DEFINES)
endif ()

##############################################################
# Dependencies
##############################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/v0.16.1/conan.cmake"
            "${CMAKE_BINARY_DIR}/conan.cmake"
            EXPECTED_HASH SHA256=396e16d0f5eabdc6a14afddbcfff62a54a7ee75c6da23f32f7a31bc85db23484
            TLS_VERIFY ON)
endif ()
include(${CMAKE_BINARY_DIR}/conan.cmake)
conan_cmake_run(CONANFILE conanfile.txt
        BASIC_SETUP
        BUILD missing)

# Don't build tests for deps
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

# Eigen required for lin alg
find_package(Eigen3 CONFIG REQUIRED)
set(EIGEN3_FOUND TRUE) # This doesn't get set by the conan generated find macro

find_package(proj CONFIG REQUIRED)
# Get Data dir in conan package
string(TOUPPER ${CMAKE_BUILD_TYPE} ${PROJECT_NAME}_UPPER_BUILD_TYPE)
set(PROJ_DATA ${PROJ_proj_RES_DIRS_${${PROJECT_NAME}_UPPER_BUILD_TYPE}})
# Set this to a compile def that is used when PROJ contexts are created
target_compile_definitions(${PROJECT_NAME} PUBLIC PROJ_DATA_PATH="${PROJ_DATA}")
target_link_libraries(${PROJECT_NAME} PUBLIC PROJ::PROJ)

find_package(OpenMP REQUIRED)
target_link_directories(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Threads REQUIRED)

find_package(netCDF CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE netCDF::netcdf)

find_package(Boost REQUIRED)

FetchContent_Declare(
        uasgroundrisk
        GIT_REPOSITORY https://github.com/aliaksei135/uasgroundrisk.git
        GIT_TAG origin/master
)
FetchContent_MakeAvailable(uasgroundrisk)
target_link_libraries(${PROJECT_NAME} PRIVATE uasgroundrisk)

# GTest should be compiled with same flags as code under test
# so using conan binaries causes strange errors
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.11.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


##############################################################
# Testing
##############################################################

set(${PROJECT_NAME}_BUILD_TESTS ON)

if (${PROJECT_NAME}_BUILD_TESTS)
    ##################################
    # GoogleTest
    # Test framework
    # License: BSD-3
    ##################################
    if (NOT TARGET gtest)
        FetchContent_Declare(
                googletest
                GIT_REPOSITORY https://github.com/google/googletest.git
                GIT_TAG release-1.11.0
        )
        # For Windows: Prevent overriding the parent project's compiler/linker settings
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif ()

    ##################################
    # matplotlib_cpp
    # C++ wrapper around matplotlib for plotting of tests
    # License: BSD-3
    ##################################
    # We do not really care about the performance of this as it is only for testing
    FetchContent_Declare(
            matplotlib_cpp
            GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
            GIT_TAG 61501081ea32549df1a02dca26cb4edbe0b6a890
    )
    FetchContent_GetProperties(matplotlib_cpp)
    if (NOT matplotlib_cpp_POPULATED)
        FetchContent_Populate(matplotlib_cpp)
        include_directories(${matplotlib_cpp_SOURCE_DIR})
    endif ()

    ##################################
    # matplot++
    # C++ data viz
    # License: MIT
    ##################################
    # We do not really care about the performance of this as it is only for testing
    #set(USE_SYSTEM_CIMG ON)
    #FetchContent_Declare(
    #    matplotplusplus
    #    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
    #    GIT_TAG origin/master
    #)
    #FetchContent_GetProperties(matplotplusplus)
    #if(NOT matplotplusplus_POPULATED)
    #    FetchContent_Populate(matplotplusplus)
    #    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
    #endif()
    #target_link_libraries(${PROJECT_NAME} PUBLIC Matplot++::matplot)

    enable_testing()
    add_subdirectory(test)
endif ()

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
                COMMENT "Buidling Doxygen documentation"
                VERBATIM)
    else (DOXYGEN_FOUND)
        message("No doxygen binary found on the system.")
        SET(${BUILD_DOC} OFF)
    endif ()
endif ()
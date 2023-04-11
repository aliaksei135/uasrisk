cmake_minimum_required(VERSION 3.16)
project(
        uasrisk-cli
        DESCRIPTION "CLI for uasrisk"
        LANGUAGES CXX
        VERSION 0.1
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CXX_STANDARD_REQUIRED ON)
Include(FetchContent)

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
else ()
    set(ENABLE_CPPCHECK ON)
endif ()
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

add_executable(uasrisk-cli
        CLI.cpp
        )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${PROJECT_BINARY_DIR})

if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/0.18.1/conan.cmake"
            "${CMAKE_BINARY_DIR}/conan.cmake"
            TLS_VERIFY ON)
endif ()
include(${CMAKE_BINARY_DIR}/conan.cmake)
conan_cmake_autodetect(settings)

if (NOT CONAN_CMD)
    find_program(CONAN_CMD conan)
    if (NOT CONAN_CMD AND CONAN_REQUIRED)
        message(FATAL_ERROR "Conan executable not found! Please install conan.")
    endif ()
endif ()
execute_process(COMMAND ${CONAN_CMD} --version
        RESULT_VARIABLE return_code
        OUTPUT_VARIABLE CONAN_VERSION_OUTPUT
        ERROR_VARIABLE CONAN_VERSION_OUTPUT)
if (NOT "${return_code}" STREQUAL "0")
    message(FATAL_ERROR "Conan --version failed='${return_code}'")
endif ()
string(REGEX MATCH ".*Conan version ([0-9]+\\.[0-9]+\\.[0-9]+)" FOO "${CONAN_VERSION_OUTPUT}")
set(CONAN_VERSION ${CMAKE_MATCH_1})
message(STATUS "Checking conan version...")
message(STATUS "Using conan: ${CONAN_CMD}")
message(STATUS "Conan version: ${CONAN_VERSION}")

# Ensure conan version is less than 2.0.0
if (CONAN_VERSION VERSION_GREATER_EQUAL "2.0.0")
    message(FATAL_ERROR "Conan version 2.0.0 or greater is not supported. Please install conan version 1.59.0 or less.")
endif ()

set(CONAN_COMMAND ${CONAN_CMD})
conan_cmake_install(PATH_OR_REFERENCE ${CMAKE_CURRENT_LIST_DIR}/conanfile.txt
        BUILD missing
        SETTINGS ${settings}
        )

# Don't build tests for deps
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

find_package(yaml-cpp CONFIG REQUIRED)
## Manually include usage requirements because CMake just doesn't feel like it
#get_target_property(yaml-cpp_INCLUDES yaml-cpp::yaml-cpp INTERFACE_INCLUDE_DIRECTORIES)
#target_include_directories(uasrisk-cli PRIVATE yaml-cpp_INCLUDES)
target_link_libraries(uasrisk-cli PRIVATE yaml-cpp)

find_package(GDAL CONFIG REQUIRED)
target_link_libraries(uasrisk-cli PRIVATE GDAL::GDAL)

find_package(pngpp CONFIG REQUIRED)
target_link_libraries(uasrisk-cli PRIVATE pngpp::pngpp)

find_package(ZLIB CONFIG REQUIRED)
target_link_libraries(uasrisk-cli PRIVATE ZLIB::ZLIB)

FetchContent_Declare(
        colormap
        GIT_REPOSITORY https://github.com/jgreitemann/colormap.git
        GIT_TAG master
)
FetchContent_GetProperties(colormap)
if (NOT colormap_POPULATED)
    FetchContent_Populate(colormap)
    target_include_directories(uasrisk-cli PUBLIC ${colormap_SOURCE_DIR}/include)
endif ()

target_link_libraries(uasrisk-cli PRIVATE uasrisk)

# Install into GNU standard locations
include(GNUInstallDirs)
install(TARGETS uasrisk-cli
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

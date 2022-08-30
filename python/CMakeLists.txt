
if(SKBUILD)
  # Scikit-Build does not add your site-packages to the search path
  # automatically, so we need to add it _or_ the pybind11 specific directory
  # here.
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
            "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE _tmp_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ECHO STDOUT)
  list(APPEND CMAKE_PREFIX_PATH "${_tmp_dir}")
endif()

pybind11_add_module(pyuasrisk 
    MODULE
    ${CMAKE_CURRENT_LIST_DIR}/Bindings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tensor.h
    )
set_target_properties(pyuasrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(uasrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(uasgroundrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

#FetchContent_Declare(
#    pybind11_tensor
#    GIT_REPOSITORY https://github.com/MattiasFredriksson/pybind11_tensor.git
#    GIT_TAG origin/main
#)
#FetchContent_GetProperties(pybind11_tensor)
#if(NOT pybind11_tensor_POPULATED)
#    FetchContent_Populate(pybind11_tensor)
#    target_include_directories(pyuasrisk PRIVATE ${pybind11_tensor_SOURCE_DIR}/include)
#endif()

target_link_libraries(pyuasrisk PRIVATE uasrisk)
target_link_libraries(pyuasrisk PRIVATE uasgroundrisk)

#include(GNUInstallDirs)
##install(TARGETS pyuasrisk DESTINATION .)
#install(TARGETS pyuasrisk
#        COMPONENT python
#        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
#        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
#        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

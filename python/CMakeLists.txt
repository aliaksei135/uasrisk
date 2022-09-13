
if (SKBUILD)
    # Scikit-Build does not add your site-packages to the search path
    # automatically, so we need to add it _or_ the pybind11 specific directory
    # here.
    execute_process(
            COMMAND "${PYTHON_EXECUTABLE}" -c
            "import pybind11; print(pybind11.get_cmake_dir())"
            OUTPUT_VARIABLE _tmp_dir
            OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ECHO STDOUT)
    list(APPEND CMAKE_PREFIX_PATH "${_tmp_dir}")
endif ()

pybind11_add_module(_pyuasrisk
        MODULE
        ${CMAKE_CURRENT_LIST_DIR}/Bindings.cpp
        ${CMAKE_CURRENT_LIST_DIR}/pybind11_eigen_tensor.h
        ${CMAKE_CURRENT_LIST_DIR}/tensor_traits.h
        )
set_target_properties(_pyuasrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(uasrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(uasgroundrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

target_link_libraries(_pyuasrisk PRIVATE uasrisk)
target_link_libraries(_pyuasrisk PRIVATE uasgroundrisk)

include(GNUInstallDirs)
install(TARGETS _pyuasrisk DESTINATION .)


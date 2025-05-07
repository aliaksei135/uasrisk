find_package(pybind11 CONFIG)


python_add_library(_pyuasrisk
        MODULE
        ${CMAKE_CURRENT_LIST_DIR}/Bindings.cpp
        ${CMAKE_CURRENT_LIST_DIR}/AirRiskHelpers.h
        ${CMAKE_CURRENT_LIST_DIR}/GroundRiskHelpers.h
        ${CMAKE_CURRENT_LIST_DIR}/pybind11_eigen_tensor.h
        ${CMAKE_CURRENT_LIST_DIR}/tensor_traits.h
        WITH_SOABI
        )
target_link_libraries(_pyuasrisk PRIVATE pybind11::headers)

set_target_properties(_pyuasrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(uasrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(uasgroundrisk PROPERTIES POSITION_INDEPENDENT_CODE TRUE)


# TODO Find a way to copy into the python package lib folder
# Add command target to copy shared libs to the python package lib folder
add_custom_command(TARGET _pyuasrisk POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:uasrisk>
        ${CMAKE_CURRENT_LIST_DIR}/pyuasrisk/lib/$<TARGET_FILE_NAME:uasrisk>)
add_custom_command(TARGET _pyuasrisk POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:uasgroundrisk>
        ${CMAKE_CURRENT_LIST_DIR}/pyuasrisk/lib/$<TARGET_FILE_NAME:uasgroundrisk>)

target_link_libraries(_pyuasrisk PRIVATE uasrisk)
target_link_libraries(_pyuasrisk PRIVATE uasgroundrisk)

include(GNUInstallDirs)
install(TARGETS _pyuasrisk DESTINATION pyuasrisk)


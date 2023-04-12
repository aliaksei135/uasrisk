include(GoogleTest)
add_custom_target(copy_test_data
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/data ${PROJECT_BINARY_DIR}/bin)

macro(ur_add_test TESTNAME)
    # create an exectuable in which the tests will be stored
    add_executable(${TESTNAME} ${TESTNAME}.cpp)

    # copy over test data files
    add_dependencies(${TESTNAME} copy_test_data)

    # link the Google test infrastructure, mocking library, and a default main fuction to
    # the test executable.  Remove g_test_main if writing your own main function.
    target_link_libraries(${TESTNAME} PUBLIC gmock_main gtest_main gtest gmock uasgroundrisk ${PROJECT_NAME})

    # gtest_discover_tests replaces gtest_add_tests,
    # see https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more options to pass to it
    gtest_discover_tests(
            ${TESTNAME}
            # set a working directory so your project root so that you can find test data via paths relative to the project root
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    )
    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()

ur_add_test(FullIngestTests)
ur_add_test(GroundRiskIntegrationTests)
ur_add_test(EmptyGridTests)
ur_add_test(VoxelGridTests)
ur_add_test(AirRiskVoxelGridTests)
ur_add_test(GroundRiskVoxelGridTests)
ur_add_test(RiskVoxelGridTests)
ur_add_test(GroundRiskIncrementalTests)

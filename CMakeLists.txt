cmake_minimum_required(VERSION 3.16)

project(RobotNav VERSION 0.5.0 LANGUAGES CXX)

# Build both sub-projects.
# Disable tests in sub-projects with -DBUILD_TESTS=OFF if desired.
option(BUILD_TESTS "Build unit tests for all sub-projects" ON)
option(BUILD_PYTHON_TESTS "Register Python unit tests with CTest" ON)
option(ROBOTNAV_REQUIRE_TEST_DEPENDENCIES
       "Fail configuration when test dependencies are unavailable" OFF)
option(BUILD_PYTHON_BINDINGS "Build pybind11 Python extension modules" OFF)
option(ROBOTNAV_ENABLE_SANITIZERS
       "Enable address and undefined-behaviour sanitizers" OFF)
set(ROBOTNAV_SANITIZERS "address;undefined" CACHE STRING
    "Semicolon-separated sanitizer list")
option(ROBOTNAV_ENABLE_CLANG_TIDY "Run clang-tidy during C++ compilation" OFF)
option(ROBOTNAV_BUILD_QUALITY_TARGETS
       "Build format and quality helper targets" ON)

if(BUILD_TESTS)
    enable_testing()
endif()

if(BUILD_TESTS AND ROBOTNAV_REQUIRE_TEST_DEPENDENCIES)
    find_package(GTest REQUIRED)
endif()

# Shared settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Quality.cmake)
robotnav_configure_quality()

include(GNUInstallDirs)
install(DIRECTORY include/robotnav DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(autoplanner)
add_subdirectory(autompc)

add_library(robotnav_pipeline STATIC
    src/dwa_local_planner.cpp
    src/dynamic_navigation_pipeline.cpp
    src/mppi_local_planner.cpp
    src/navigation_pipeline.cpp
    src/navigation_trace.cpp
    src/safety_supervisor.cpp
    src/scenario_config.cpp
    src/space_time_astar.cpp
)
target_include_directories(robotnav_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(robotnav_pipeline PUBLIC autoplanner autompc)
target_compile_options(robotnav_pipeline PRIVATE -Wall -Wextra -Wpedantic)

# MPPI rollouts are independent. Use OpenMP when it is available, while
# keeping the default build portable on machines without an OpenMP runtime.
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    target_link_libraries(robotnav_pipeline PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(robotnav_pipeline PRIVATE ROBOTNAV_HAS_OPENMP=1)
endif()

install(TARGETS robotnav_pipeline
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_executable(dynamic_navigation apps/dynamic_navigation.cpp)
target_link_libraries(dynamic_navigation PRIVATE autoplanner autompc)
set_target_properties(dynamic_navigation PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

add_executable(navigation_pipeline_cli apps/navigation_pipeline_cli.cpp)
target_link_libraries(navigation_pipeline_cli PRIVATE robotnav_pipeline)
set_target_properties(navigation_pipeline_cli PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

add_executable(dynamic_navigation_pipeline_cli
    apps/dynamic_navigation_pipeline_cli.cpp)
target_link_libraries(dynamic_navigation_pipeline_cli PRIVATE robotnav_pipeline)
set_target_properties(dynamic_navigation_pipeline_cli PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

if(BUILD_TESTS)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        add_subdirectory(tests)
    else()
        if(ROBOTNAV_REQUIRE_TEST_DEPENDENCIES)
            message(FATAL_ERROR "GTest is required but was not found")
        endif()
        message(STATUS "GTest not found; skipping RobotNav integration tests")
    endif()
endif()

# Python tests are deliberately registered individually.  The core Python
# helpers use only the standard library, while dashboard tests additionally
# require optional YAML/Plotly/Streamlit dependencies.  This keeps the normal
# C++ test suite usable in a minimal environment without silently omitting the
# tests that can run there.
if(BUILD_TESTS AND BUILD_PYTHON_TESTS)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        function(robotnav_add_python_test name source)
            add_test(NAME ${name}
                     COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/${source}
                             ${ARGN}
                     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
            set_tests_properties(${name} PROPERTIES
                                 LABELS "python"
                                 TIMEOUT 60)
        endfunction()

        robotnav_add_python_test(test_python_perception_mapping
                                 autoplanner/tests/test_perception_mapping.py)
        robotnav_add_python_test(test_python_local_planner_benchmark
                                 autoplanner/tests/test_local_planner_benchmark.py)
        if(BUILD_PYTHON_BINDINGS)
            robotnav_add_python_test(test_python_robotnav_facade
                                     robotnav/tests/test_planning.py)
            set_tests_properties(test_python_robotnav_facade PROPERTIES
                                 ENVIRONMENT
                                 "PYTHONPATH=${CMAKE_BINARY_DIR}/python"
                                 TIMEOUT 60)
        endif()
        robotnav_add_python_test(
            test_python_perception_replay
            autoplanner/scripts/run_perception_pipeline.py
            --map ${CMAKE_SOURCE_DIR}/autoplanner/data/maps/simple_50x50.txt
            --start 1 1
            --goal 12 12
            --frames 6
            --dynamic-obstacle moving 7 4.5 0 0.5 0.6
            --cpp-build-dir ${CMAKE_BINARY_DIR}
            --cpp-planner space_time_astar
            --cpp-local-planner none
            --output-dir ${CMAKE_BINARY_DIR}/perception-replay)
        set_tests_properties(test_python_perception_replay PROPERTIES
                             LABELS "python;integration"
                             TIMEOUT 120)

        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c
                    "import yaml, plotly, streamlit"
            RESULT_VARIABLE ROBOTNAV_DASHBOARD_IMPORT_RESULT
            OUTPUT_QUIET ERROR_QUIET)
        if(ROBOTNAV_DASHBOARD_IMPORT_RESULT EQUAL 0)
            robotnav_add_python_test(test_python_dashboard_support
                                     autoplanner/tests/test_dashboard_support.py)
        else()
            message(STATUS
                    "Dashboard Python dependencies not found; skipping dashboard tests")
        endif()
    elseif(ROBOTNAV_REQUIRE_TEST_DEPENDENCIES)
        message(FATAL_ERROR "Python3 interpreter is required but was not found")
    else()
        message(STATUS "Python3 interpreter not found; skipping Python tests")
    endif()
endif()

if(BUILD_PYTHON_BINDINGS)
    # The facade is pure Python and lives next to the compiled extension
    # modules in the build tree, so PYTHONPATH=build/python is sufficient for
    # source-tree binding smoke tests.
    file(COPY "${CMAKE_SOURCE_DIR}/robotnav/python/robotnav"
         DESTINATION "${CMAKE_BINARY_DIR}/python")
    install(DIRECTORY robotnav/python/robotnav
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/python)
endif()

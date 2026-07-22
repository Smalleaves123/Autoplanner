cmake_minimum_required(VERSION 3.16)

project(RobotNav VERSION 0.5.0 LANGUAGES CXX)

# Build both sub-projects.
# Disable tests in sub-projects with -DBUILD_TESTS=OFF if desired.
option(BUILD_TESTS "Build unit tests for all sub-projects" ON)
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
    src/dynamic_navigation_pipeline.cpp
    src/navigation_pipeline.cpp
    src/navigation_trace.cpp
    src/safety_supervisor.cpp
    src/scenario_config.cpp
)
target_include_directories(robotnav_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(robotnav_pipeline PUBLIC autoplanner autompc)
target_compile_options(robotnav_pipeline PRIVATE -Wall -Wextra -Wpedantic)

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
        message(STATUS "GTest not found; skipping RobotNav integration tests")
    endif()
endif()

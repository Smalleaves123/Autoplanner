cmake_minimum_required(VERSION 3.16)

project(RobotNav VERSION 0.5.0 LANGUAGES CXX)

# Build both sub-projects.
# Disable tests in sub-projects with -DBUILD_TESTS=OFF if desired.
option(BUILD_TESTS "Build unit tests for all sub-projects" ON)
option(BUILD_PYTHON_BINDINGS "Build pybind11 Python extension modules" OFF)

if(BUILD_TESTS)
    enable_testing()
endif()

# Shared settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(autoplanner)
add_subdirectory(autompc)

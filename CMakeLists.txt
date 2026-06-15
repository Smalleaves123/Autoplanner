cmake_minimum_required(VERSION 3.16)

project(RobotNav VERSION 0.5.0 LANGUAGES CXX)

# Build both sub-projects.
# Disable tests in sub-projects with -DBUILD_TESTS=OFF if desired.
option(BUILD_TESTS "Build unit tests for all sub-projects" ON)

# Shared settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(autoplanner)
add_subdirectory(autompc)

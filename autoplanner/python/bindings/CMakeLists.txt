cmake_minimum_required(VERSION 3.16)

include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.12.0
)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(_autoplanner py_autoplanner.cpp)

target_link_libraries(_autoplanner PRIVATE autoplanner)

target_include_directories(_autoplanner PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

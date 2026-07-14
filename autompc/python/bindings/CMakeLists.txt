find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYBIND11_RESULT
)
if(NOT PYBIND11_RESULT EQUAL 0)
    message(FATAL_ERROR "Python pybind11 package is required for BUILD_PYTHON_BINDINGS")
endif()

list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_autompc py_autompc.cpp)
target_link_libraries(_autompc PRIVATE autompc)

set(PYTHON_PACKAGE_DIR "${CMAKE_BINARY_DIR}/python/autompc")
file(MAKE_DIRECTORY "${PYTHON_PACKAGE_DIR}")
set_target_properties(_autompc PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PYTHON_PACKAGE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${PYTHON_PACKAGE_DIR}"
)
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/../autompc/__init__.py"
    "${PYTHON_PACKAGE_DIR}/__init__.py"
    COPYONLY
)

install(TARGETS _autompc
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python/autompc
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/python/autompc
)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../autompc/__init__.py"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/python/autompc
)

cmake_minimum_required(VERSION 3.16)

project(AutoMPC VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────

# Eigen (optional, for LQR controller)
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    add_compile_definitions(AUTOMPC_HAS_EIGEN)
endif()

# Google Test (for tests)
find_package(GTest QUIET)

# ── Core library ────────────────────────────────────────────────────────

set(AUTOMPC_SOURCES
    src/core/trajectory.cpp
    src/controllers/pid_controller.cpp
    src/trajectory/trajectory_tracker.cpp
    src/trajectory/error_metrics.cpp
)

if(Eigen3_FOUND)
    list(APPEND AUTOMPC_SOURCES src/controllers/lqr_controller.cpp)
endif()

add_library(autompc STATIC ${AUTOMPC_SOURCES})

target_include_directories(autompc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(Eigen3_FOUND)
    target_link_libraries(autompc PUBLIC Eigen3::Eigen)
endif()

target_compile_options(autompc PRIVATE -Wall -Wextra -Wpedantic)

# ── CLI app ─────────────────────────────────────────────────────────────

add_executable(autompc_cli apps/autompc_cli.cpp)
target_link_libraries(autompc_cli PRIVATE autompc)
set_target_properties(autompc_cli PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

# ── Examples ────────────────────────────────────────────────────────────

add_executable(circle_tracking examples/circle_tracking.cpp)
target_link_libraries(circle_tracking PRIVATE autompc)
set_target_properties(circle_tracking PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
)

# ── Tests ───────────────────────────────────────────────────────────────

if(GTest_FOUND)
    enable_testing()
    function(add_mpc_test name source)
        add_executable(${name} ${source})
        target_link_libraries(${name} PRIVATE autompc GTest::GTest GTest::Main)
        target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        set_target_properties(${name} PROPERTIES BUILD_RPATH "/opt/anaconda3/lib")
        add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        set_tests_properties(${name} PROPERTIES
            ENVIRONMENT "DYLD_LIBRARY_PATH=/opt/anaconda3/lib")
    endfunction()

    add_mpc_test(test_controllers tests/test_controllers.cpp)
    add_mpc_test(test_edge_cases tests/test_edge_cases.cpp)
endif()

# ── Install ─────────────────────────────────────────────────────────────

include(GNUInstallDirs)

install(TARGETS autompc autompc_cli circle_tracking
    EXPORT autompcTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/autompc
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT autompcTargets
    NAMESPACE autompc::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/autompc
)

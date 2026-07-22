# Shared engineering-quality switches for the RobotNav monorepo.

function(robotnav_configure_quality)
    if(ROBOTNAV_ENABLE_SANITIZERS)
        if(MSVC)
            message(FATAL_ERROR "RobotNav sanitizers are currently supported on non-MSVC toolchains only")
        endif()

        string(REPLACE ";" "," ROBOTNAV_SANITIZER_LIST
               "${ROBOTNAV_SANITIZERS}")
        add_compile_options(
            "-fsanitize=${ROBOTNAV_SANITIZER_LIST}"
            -fno-omit-frame-pointer
        )
        add_link_options("-fsanitize=${ROBOTNAV_SANITIZER_LIST}")
        message(STATUS "RobotNav sanitizers: ${ROBOTNAV_SANITIZERS}")
    endif()

    if(ROBOTNAV_ENABLE_CLANG_TIDY)
        find_program(ROBOTNAV_CLANG_TIDY_EXECUTABLE NAMES clang-tidy)
        if(NOT ROBOTNAV_CLANG_TIDY_EXECUTABLE)
            message(FATAL_ERROR
                    "ROBOTNAV_ENABLE_CLANG_TIDY=ON but clang-tidy was not found")
        endif()
        set(CMAKE_CXX_CLANG_TIDY
            "${ROBOTNAV_CLANG_TIDY_EXECUTABLE};-p=${CMAKE_BINARY_DIR}"
            CACHE STRING "clang-tidy command" FORCE)
        message(STATUS "RobotNav clang-tidy: ${ROBOTNAV_CLANG_TIDY_EXECUTABLE}")
    endif()

    if(NOT ROBOTNAV_BUILD_QUALITY_TARGETS)
        return()
    endif()

    find_program(ROBOTNAV_CLANG_FORMAT_EXECUTABLE NAMES clang-format)
    file(GLOB_RECURSE ROBOTNAV_FORMAT_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_SOURCE_DIR}/*.h"
         "${CMAKE_SOURCE_DIR}/*.hpp"
         "${CMAKE_SOURCE_DIR}/*.cpp")
    list(FILTER ROBOTNAV_FORMAT_SOURCES EXCLUDE REGEX "/build[^/]*/")
    list(FILTER ROBOTNAV_FORMAT_SOURCES EXCLUDE REGEX "/results/")
    list(FILTER ROBOTNAV_FORMAT_SOURCES EXCLUDE REGEX "/ros2_ws/")

    if(ROBOTNAV_CLANG_FORMAT_EXECUTABLE AND ROBOTNAV_FORMAT_SOURCES)
        add_custom_target(format-check
            COMMAND "${ROBOTNAV_CLANG_FORMAT_EXECUTABLE}"
                    --dry-run --Werror ${ROBOTNAV_FORMAT_SOURCES}
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            COMMENT "Checking C++ formatting with clang-format"
            VERBATIM
        )
        add_custom_target(format
            COMMAND "${ROBOTNAV_CLANG_FORMAT_EXECUTABLE}"
                    -i ${ROBOTNAV_FORMAT_SOURCES}
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            COMMENT "Formatting C++ sources with clang-format"
            VERBATIM
        )
    else()
        add_custom_target(format-check
            COMMAND "${CMAKE_COMMAND}" -E echo
                    "clang-format not found. Install it to run format-check"
            COMMENT "C++ formatting check unavailable"
        )
    endif()

    add_custom_target(quality-check DEPENDS format-check)
endfunction()

cmake_minimum_required(VERSION 3.16)

project(AutoPlanner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ── Core library ────────────────────────────────────────────────────────

add_library(autoplanner STATIC
    # core
    src/core/grid_map.cpp
    src/core/path.cpp
    src/core/planner_factory.cpp
    src/core/planner_result.cpp
    src/core/types.cpp

    # graph search planners
    src/planners/graph_search/astar.cpp
    src/planners/graph_search/dijkstra.cpp
    src/planners/graph_search/weighted_astar.cpp
    src/planners/graph_search/improved_astar.cpp
    src/planners/graph_search/jps.cpp
    src/planners/graph_search/dstar_lite.cpp

    # sampling planners
    src/planners/sampling/rrt.cpp
    src/planners/sampling/rrt_star.cpp
    src/planners/sampling/informed_rrt_star.cpp
    src/planners/sampling/bi_rrt.cpp

    # kinodynamic planners
    src/planners/kinodynamic/hybrid_astar.cpp

    # costmap
    src/costmap/costmap_2d.cpp
    src/costmap/obstacle_inflation.cpp
    src/costmap/distance_transform.cpp
    src/costmap/obstacle_cost_layer.cpp

    # collision
    src/collision/collision_checker.cpp
    src/collision/footprint_collision_checker.cpp
    src/collision/grid_collision_checker.cpp
    src/collision/line_collision_checker.cpp

    # smoothing
    src/smoothing/shortcut_smoother.cpp
    src/smoothing/bezier_smoother.cpp
    src/smoothing/bspline_smoother.cpp
    src/smoothing/gradient_smoother.cpp

    # metrics
    src/metrics/path_metrics.cpp
    src/metrics/benchmark_metrics.cpp
    src/metrics/planner_profiler.cpp

    # heuristics
    src/heuristics/euclidean.cpp
    src/heuristics/manhattan.cpp
    src/heuristics/diagonal.cpp
    src/heuristics/reeds_shepp.cpp

    # io
    src/io/config_loader.cpp
    src/io/map_loader.cpp
    src/io/map_saver.cpp
    src/io/path_writer.cpp
    src/io/json_writer.cpp

    # utilities
    src/utils/logger.cpp
    src/utils/math_utils.cpp
    src/utils/random.cpp
    src/utils/timer.cpp
)

target_include_directories(autoplanner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_options(autoplanner PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# ── CLI apps ────────────────────────────────────────────────────────────

function(add_app name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE autoplanner)
    set_target_properties(${name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
    )
endfunction()

add_app(run_single_planner apps/run_single_planner.cpp)
add_app(autoplanner_cli    apps/autoplanner_cli.cpp)
add_app(compare_planners   apps/compare_planners.cpp)
add_app(ablation_study     apps/ablation_study.cpp)
add_app(dynamic_replanning apps/dynamic_replanning.cpp)

# ── Examples ────────────────────────────────────────────────────────────

add_executable(astar_example           examples/astar_example.cpp)
add_executable(improved_astar_example  examples/improved_astar_example.cpp)
add_executable(rrt_example             examples/rrt_example.cpp)
add_executable(rrt_star_example        examples/rrt_star_example.cpp)
add_executable(smoothing_example       examples/smoothing_example.cpp)
add_executable(costmap_example         examples/costmap_example.cpp)

set(EXAMPLE_TARGETS
    astar_example improved_astar_example rrt_example
    rrt_star_example smoothing_example costmap_example
)

foreach(target ${EXAMPLE_TARGETS})
    target_link_libraries(${target} PRIVATE autoplanner)
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )
endforeach()

# ── Benchmark app ───────────────────────────────────────────────────────

add_executable(benchmark_all benchmark/benchmark_all.cpp)
target_link_libraries(benchmark_all PRIVATE autoplanner)
set_target_properties(benchmark_all PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmark
)

# ── Install ──────────────────────────────────────────────────────────────

include(GNUInstallDirs)

install(TARGETS autoplanner
    EXPORT autoplannerTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/autoplanner
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export targets so other projects can use find_package(autoplanner)
install(EXPORT autoplannerTargets
    NAMESPACE autoplanner::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/autoplanner
)

# ── Tests ───────────────────────────────────────────────────────────────

option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    find_package(GTest QUIET)
endif()

if(BUILD_TESTS AND GTest_FOUND)
    enable_testing()
    add_subdirectory(tests)
elseif(BUILD_TESTS)
    message(WARNING "GoogleTest not found; AutoPlanner tests will not be built")
endif()

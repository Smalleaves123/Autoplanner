find_package(GTest REQUIRED)

# ── Test executables ──────────────────────────────────────────────────────

function(add_autoplanner_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE autoplanner GTest::GTest GTest::Main)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic)
    # Set rpath so the binary can find GTest dylibs at runtime.
    set_target_properties(${name} PROPERTIES
        BUILD_RPATH "/opt/anaconda3/lib"
    )
    add_test(
        NAME ${name}
        COMMAND ${name}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    )
    # macOS needs DYLD_LIBRARY_PATH for conda-installed GTest.
    set_tests_properties(${name} PROPERTIES
        ENVIRONMENT "DYLD_LIBRARY_PATH=/opt/anaconda3/lib"
    )
endfunction()

add_autoplanner_test(test_grid_map          test_grid_map.cpp)
add_autoplanner_test(test_map_loader        test_map_loader.cpp)
add_autoplanner_test(test_astar             test_astar.cpp)
add_autoplanner_test(test_dijkstra          test_dijkstra.cpp)
add_autoplanner_test(test_improved_astar    test_improved_astar.cpp)
add_autoplanner_test(test_collision_checker test_collision_checker.cpp)
add_autoplanner_test(test_rrt               test_rrt.cpp)
add_autoplanner_test(test_rrt_star          test_rrt_star.cpp)
add_autoplanner_test(test_smoothing         test_smoothing.cpp)
add_autoplanner_test(test_metrics           test_metrics.cpp)
add_autoplanner_test(test_config_loader     test_config_loader.cpp)
add_autoplanner_test(test_utils             test_utils.cpp)
add_autoplanner_test(test_edge_cases        test_edge_cases.cpp)

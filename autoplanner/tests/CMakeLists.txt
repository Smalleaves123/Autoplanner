find_package(GTest REQUIRED)

# ── Test executables ──────────────────────────────────────────────────────

function(add_autoplanner_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE autoplanner GTest::GTest GTest::Main)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic)
    set(gtest_runtime_target "")
    if(TARGET GTest::gtest_main)
        set(gtest_runtime_target "GTest::gtest_main")
    elseif(TARGET GTest::Main)
        get_target_property(gtest_main_type GTest::Main TYPE)
        if(NOT gtest_main_type STREQUAL "INTERFACE_LIBRARY")
            set(gtest_runtime_target "GTest::Main")
        endif()
    endif()
    if(gtest_runtime_target)
        # Keep the test binary relocatable while allowing package-managed
        # shared GTest libraries to be found on macOS and Linux.
        set_property(TARGET ${name} APPEND PROPERTY BUILD_RPATH
                     "$<TARGET_FILE_DIR:${gtest_runtime_target}>")
    endif()
    add_test(
        NAME ${name}
        COMMAND ${name}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    )
endfunction()

add_autoplanner_test(test_grid_map          test_grid_map.cpp)
add_autoplanner_test(test_map_loader        test_map_loader.cpp)
add_autoplanner_test(test_astar             test_astar.cpp)
add_autoplanner_test(test_dijkstra          test_dijkstra.cpp)
add_autoplanner_test(test_improved_astar    test_improved_astar.cpp)
add_autoplanner_test(test_weighted_astar    test_weighted_astar.cpp)
add_autoplanner_test(test_jps               test_jps.cpp)
add_autoplanner_test(test_dstar_lite        test_dstar_lite.cpp)
add_autoplanner_test(test_collision_checker test_collision_checker.cpp)
add_autoplanner_test(test_planner_factory   test_planner_factory.cpp)
add_autoplanner_test(test_rrt               test_rrt.cpp)
add_autoplanner_test(test_rrt_star          test_rrt_star.cpp)
add_autoplanner_test(test_informed_rrt_star test_informed_rrt_star.cpp)
add_autoplanner_test(test_bi_rrt            test_bi_rrt.cpp)
add_autoplanner_test(test_hybrid_astar      test_hybrid_astar.cpp)
add_autoplanner_test(test_smoothing         test_smoothing.cpp)
add_autoplanner_test(test_metrics           test_metrics.cpp)
add_autoplanner_test(test_config_loader     test_config_loader.cpp)
add_autoplanner_test(test_utils             test_utils.cpp)
add_autoplanner_test(test_edge_cases        test_edge_cases.cpp)

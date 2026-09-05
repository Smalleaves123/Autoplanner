find_package(GTest REQUIRED)

function(add_robotnav_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE
        robotnav_pipeline GTest::GTest GTest::Main)
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic)

    if(TARGET GTest::gtest_main)
        set_property(TARGET ${name} APPEND PROPERTY BUILD_RPATH
                     "$<TARGET_FILE_DIR:GTest::gtest_main>")
    elseif(TARGET GTest::Main)
        get_target_property(gtest_main_type GTest::Main TYPE)
        if(NOT gtest_main_type STREQUAL "INTERFACE_LIBRARY")
            set_property(TARGET ${name} APPEND PROPERTY BUILD_RPATH
                         "$<TARGET_FILE_DIR:GTest::Main>")
        endif()
    endif()

    add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endfunction()

add_robotnav_test(test_navigation_pipeline test_navigation_pipeline.cpp)
add_robotnav_test(test_trajectory_controller test_trajectory_controller.cpp)
add_robotnav_test(test_local_planner_registry test_local_planner_registry.cpp)
add_robotnav_test(test_component_catalog test_component_catalog.cpp)

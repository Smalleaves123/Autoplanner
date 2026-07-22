find_package(GTest REQUIRED)

add_executable(test_navigation_pipeline test_navigation_pipeline.cpp)
target_link_libraries(test_navigation_pipeline PRIVATE
    robotnav_pipeline GTest::GTest GTest::Main)
target_include_directories(test_navigation_pipeline PRIVATE
    ${CMAKE_SOURCE_DIR}/include)
target_compile_options(test_navigation_pipeline PRIVATE -Wall -Wextra -Wpedantic)

if(TARGET GTest::gtest_main)
    set_property(TARGET test_navigation_pipeline APPEND PROPERTY BUILD_RPATH
                 "$<TARGET_FILE_DIR:GTest::gtest_main>")
elseif(TARGET GTest::Main)
    get_target_property(gtest_main_type GTest::Main TYPE)
    if(NOT gtest_main_type STREQUAL "INTERFACE_LIBRARY")
        set_property(TARGET test_navigation_pipeline APPEND PROPERTY BUILD_RPATH
                     "$<TARGET_FILE_DIR:GTest::Main>")
    endif()
endif()

add_test(NAME test_navigation_pipeline
         COMMAND test_navigation_pipeline
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

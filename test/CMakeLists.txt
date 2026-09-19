# Unit tests for mod. Data files live in test/data; MOD_TEST_DATA_DIR points at it.
include(GoogleTest)

add_library(mod_test_common INTERFACE)
target_compile_definitions(mod_test_common INTERFACE MOD_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_link_libraries(mod_test_common INTERFACE mod::mod GTest::gtest GTest::gtest_main)
target_compile_options(mod_test_common INTERFACE -Wall -Wextra)

# Test executables go to build/bin/tests, apart from the tools in build/bin.
set(MOD_TEST_BIN_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tests)

function(mod_add_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE mod_test_common)
  set_target_properties(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${MOD_TEST_BIN_DIR})
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endfunction()

mod_add_test(parameters_test)
mod_add_test(maps_test)
mod_add_test(objective_test)
mod_add_test(dijkstra_sampler_test)
mod_add_test(intensity_sampler_test)
mod_add_test(hybrid_sampler_test)
mod_add_test(timing_test)

if(TARGET mod_playground)
  foreach(name occupancy_map_test footprint_test run_logger_test hybrid_astar_test)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE mod_test_common mod::playground)
    set_target_properties(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${MOD_TEST_BIN_DIR})
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
  endforeach()
endif()

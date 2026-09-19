# The playground: headless core library, batch runner and tools (M4); ImGui app (M5).
add_library(mod_playground STATIC
  core/occupancy_map.cpp
  core/footprint_checker.cpp
  core/map_cache.cpp
  core/planner_factory.cpp
  core/solver.cpp
  core/run_logger.cpp
  core/batch.cpp)
add_library(mod::playground ALIAS mod_playground)
target_include_directories(mod_playground PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(mod_playground PUBLIC mod::mod)
target_compile_options(mod_playground PRIVATE -Wall -Wextra)
set_target_properties(mod_playground PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)

add_executable(run_batch tools/run_batch.cpp)
target_link_libraries(run_batch PRIVATE mod::playground Threads::Threads)
target_compile_options(run_batch PRIVATE -Wall -Wextra)

add_executable(check_interpolation tools/check_interpolation.cpp)
target_link_libraries(check_interpolation PRIVATE mod::playground)
target_compile_options(check_interpolation PRIVATE -Wall -Wextra)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/gui/CMakeLists.txt)
  add_subdirectory(gui)
endif()

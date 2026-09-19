cmake_minimum_required(VERSION 3.16)
project(mod VERSION 2.0.0 LANGUAGES CXX)
cmake_policy(SET CMP0167 NEW)  # use BoostConfig.cmake, not the removed FindBoost module

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Flat build tree: executables in build/bin, libraries in build/lib.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(MOD_BUILD_TESTS "Build the mod unit tests" ON)
option(MOD_BUILD_PLAYGROUND "Build the mod playground (batch runner, tools, GUI)" ON)

find_package(Boost REQUIRED)  # header-only: property_tree (XML), geometry (rtree)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(nlohmann_json REQUIRED)

message(STATUS "mod: OMPL ${ompl_VERSION} at ${OMPL_INCLUDE_DIRS}")

# Version header (git describe baked in at configure time).
find_package(Git QUIET)
set(MOD_GIT_HASH "unknown")
if(GIT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --always --dirty --tags
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE MOD_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/mod_version.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/generated/mod/version.h @ONLY)

###########
## Build ##
###########

add_library(mod SHARED
  src/cliffmap.cpp
  src/gmmtmap.cpp
  src/parameters.cpp
  src/ompl/mod/objectives/MoDOptimizationObjective.cpp
  src/ompl/mod/objectives/IntensityMapOptimizationObjective.cpp
  src/ompl/mod/objectives/UpstreamCriterionOptimizationObjective.cpp
  src/ompl/mod/objectives/DTCOptimizationObjective.cpp
  src/ompl/mod/samplers/IntensityMapSampler.cpp
  src/ompl/mod/samplers/DijkstraSampler.cpp
  src/ompl/mod/samplers/HybridSampler.cpp)
add_library(mod::mod ALIAS mod)

target_include_directories(mod PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mod PUBLIC
  ompl::ompl
  Eigen3::Eigen
  Boost::headers
  nlohmann_json::nlohmann_json)
target_compile_options(mod PRIVATE -Wall -Wextra)
set_target_properties(mod PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})

if(MOD_BUILD_PLAYGROUND)
  add_subdirectory(src/playground playground)
endif()

if(MOD_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  add_subdirectory(test)
endif()

#############
## Install ##
#############
include(GNUInstallDirs)

install(TARGETS mod EXPORT modTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/generated/mod/version.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mod)

install(EXPORT modTargets
  NAMESPACE mod::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/mod/cmake)

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/modConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/modConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/mod/cmake
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/modConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/modConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/modConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/mod/cmake)

########### Add uninstall target ###############
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")

include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/CPackSettings.cmake)

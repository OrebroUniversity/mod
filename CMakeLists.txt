cmake_minimum_required(VERSION 3.0)
project(mod VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(Boost REQUIRED COMPONENTS log)
find_package(Eigen3 REQUIRED)
find_package(ompl 1.5 REQUIRED)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIR}
)

## Declare a C++ library
add_library(mod SHARED
        src/cliffmap.cpp
        src/gmmtmap.cpp
        src/ompl/mod/objectives/IntensityMapOptimizationObjective.cpp
        src/ompl/mod/objectives/UpstreamCriterionOptimizationObjective.cpp
        src/ompl/mod/objectives/DTCOptimizationObjective.cpp
        src/ompl/mod/samplers/IntensityMapSampler.cpp)
target_link_libraries(mod ${Boost_LIBRARIES} ${ompl_LIBRARIES})

#############
## Install ##
#############
include(GNUInstallDirs)

install(TARGETS mod
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )

include(CMakePackageConfigHelpers)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/modConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/modConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/mod/cmake
        PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
        NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/modConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/modConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/modConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/mod/cmake
        COMPONENT mod)

########### Add uninstall target ###############
CONFIGURE_FILE(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
        "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")

include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/CPackSettings.cmake)

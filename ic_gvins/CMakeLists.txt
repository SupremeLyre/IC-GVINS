cmake_minimum_required(VERSION 3.10)
project(ic_gvins)

set(ROS_VERSION "$ENV{ROS_VERSION}" CACHE STRING "ROS major version (1 or 2)")
set_property(CACHE ROS_VERSION PROPERTY STRINGS 1 2)
if(NOT ROS_VERSION)
    find_package(catkin QUIET COMPONENTS roscpp)
    if(catkin_FOUND)
        set(ROS_VERSION 1 CACHE STRING "ROS major version (1 or 2)" FORCE)
    else()
        find_package(ament_cmake REQUIRED)
        set(ROS_VERSION 2 CACHE STRING "ROS major version (1 or 2)" FORCE)
    endif()
endif()
if(NOT ROS_VERSION MATCHES "^[12]$")
    message(FATAL_ERROR "ROS_VERSION must be 1 or 2; source the desired ROS setup file before building")
endif()
message(STATUS "Building IC-GVINS for ROS ${ROS_VERSION}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Optimization
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Build GVINS library
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/ic_gvins)
file(GLOB_RECURSE GVINS_SOURCE
        ic_gvins/*.cc
        ic_gvins/common/*.cc
        ic_gvins/fileio/*.cc
        ic_gvins/tracking/*.cc
        ic_gvins/preintegration/*.cc)
add_library(ic_gvins_core SHARED
        ${GVINS_SOURCE})
set_target_properties(ic_gvins_core PROPERTIES INSTALL_RPATH "$ORIGIN")

# OpenCV
find_package(OpenCV REQUIRED)
target_link_libraries(ic_gvins_core ${OpenCV_LIBS})

# glog
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
target_link_libraries(ic_gvins_core ${GLOG_LIBRARIES})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(ic_gvins_core ${YAML_CPP_LIBRARIES})

# Ceres
find_package(Ceres 2.0 REQUIRED)
target_link_libraries(ic_gvins_core ${CERES_LIBRARIES})

# boost
find_package(Boost REQUIRED COMPONENTS filesystem)
target_link_libraries(ic_gvins_core
        ${Boost_LIBRARIES})

# TBB
find_package(TBB REQUIRED)
target_link_libraries(ic_gvins_core
        TBB::tbb)

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
# Keep the bundled libraries static even when ament sets BUILD_SHARED_LIBS.
function(add_gvins_abseil)
    set(BUILD_SHARED_LIBS OFF)
    add_subdirectory(thirdparty/abseil-cpp abseil-cpp)
endfunction()
add_gvins_abseil()
target_link_libraries(ic_gvins_core
        absl::strings
        absl::str_format
        absl::time)

# Separate ROS build systems, following the OpenVINS layout.
include(cmake/ROS${ROS_VERSION}.cmake)

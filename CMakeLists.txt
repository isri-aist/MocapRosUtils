cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME mocap_ros_utils)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/MocapRosUtils)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT FALSE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
set(DOXYGEN_HTML_OUTPUT html)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(${PROJECT_NAME} LANGUAGES CXX)

# RBDyn
find_package(RBDyn REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  INCLUDE_DIRS
  include
  LIBRARIES
  MocapRosUtils)

add_subdirectory(src)

add_subdirectory(node)

# if(INSTALL_DOCUMENTATION) add_subdirectory(doc) endif()

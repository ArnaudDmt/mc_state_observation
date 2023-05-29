cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME mc_state_observation)
set(PROJECT_DESCRIPTION "Implementation of state-observation for mc_rtc")
set(PROJECT_URL "https://github.com/isri-aist/mc_state_observation")
set(PROJECT_VERSION 1.0.3)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(DOXYGEN_USE_MATHJAX "YES")
set(CMAKE_CXX_STANDARD 14)
set(CXX_DISABLE_WERROR 1)

include(cmake/base.cmake)


  set(PROJECT_USE_CMAKE_EXPORT TRUE)

project(${PROJECT_NAME} LANGUAGES CXX)

if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
  set(WITH_ROS_OBSERVERS_DEFAULT ON)
else()
  set(WITH_ROS_OBSERVERS_DEFAULT OFF)
endif()
option(WITH_ROS_OBSERVERS "Enable ROS-based observers" ${WITH_ROS_OBSERVERS_DEFAULT})

find_package(mc_rtc REQUIRED)
find_package(state-observation 1.3 REQUIRED)

if(WITH_ROS_OBSERVERS AND NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required to build ROS-based observers")
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)
find_package(mc_state_observation_ROS COMPONENTS roscpp tf2 tf2_eigen)

add_subdirectory(src)

if(BUILD_TESTING)
  find_package(Boost COMPONENTS unit_test_framework REQUIRED)
  add_subdirectory(tests)
endif()

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

################################################################
#    Coman project header. Don't touch unless you know how     #
################################################################
set(ENV{COMAN_ROOT} "~/COMAN_shared")
IF("$ENV{COMAN_ROOT}" STREQUAL "")
  message(FATAL_ERROR "!!! No COMAN_ROOT environment variable. !!!\n"
          "Try: >export COMAN_ROOT=/location/of/coman/libraries\n" $ENV{COMAN_ROOT})
ELSE()
  message("Install root is: " $ENV{COMAN_ROOT})
  set(COMAN_ROOT $ENV{COMAN_ROOT})
ENDIF()

#Install into source folder unless user sets something different
SET(MY_INSTALL_PREFIX ${COMAN_ROOT} CACHE PATH "Prefix prepended to install directories")
SET(CMAKE_INSTALL_PREFIX "${MY_INSTALL_PREFIX}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${COMAN_ROOT}/utils/cmake"
  "${COMAN_ROOT}/utils/cmake/modules" #Local Find* modules
  )

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories("${COMAN_ROOT}/src/robolli" "${COMAN_ROOT}/src/microstrain" "${COMAN_ROOT}/src/os")
link_directories("${COMAN_ROOT}/examples/lib")


add_definitions("-std=c++0x")

#add_definitions(-DRT_ENV -D__XENO__) #<-- RT
#include_directories("/usr/local/rtnet/include") #<-- RT
################################################################
#                                                              #
################################################################

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(coman_state_publisher src/coman_state_publisher.cpp)
#target_link_libraries(example ${PROJECT_NAME})

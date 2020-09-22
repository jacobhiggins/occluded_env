#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( MY_COOL_PROJECT CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
ADD_DEFINITIONS(-std=c++11)
ADD_EXECUTABLE(        vel_cmd_jackal3 vel_cmd_jackal3.cpp )
#ADD_EXECUTABLE(        vel_cmd vel_cmd.cpp )
TARGET_LINK_LIBRARIES( vel_cmd_jackal3 ${ACADO_SHARED_LIBRARIES} )
#TARGET_LINK_LIBRARIES( vel_cmd ${ACADO_SHARED_LIBRARIES} )
SET_TARGET_PROPERTIES( vel_cmd_jackal3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
#SET_TARGET_PROPERTIES( vel_cmd PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
